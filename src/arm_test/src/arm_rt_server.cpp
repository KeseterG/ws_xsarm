#include "arm_test/arm_rt_server.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <arm_interfaces/srv/detail/rt_command__struct.hpp>
#include <control_msgs/msg/detail/joint_jog__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <memory>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/detail/planning_scene__struct.hpp>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <string>

namespace urc_arm::server
{
ArmRTServer::ArmRTServer(const rclcpp::NodeOptions& options) : rclcpp::Node("arm_rtpose_server", options)
{
  RCLCPP_INFO(get_logger(), "Starting Initializing Arm RT Pose server...");
  if (!options.use_intra_process_comms())
  {
    RCLCPP_WARN_STREAM(get_logger(),
                       "Intra-process communication is disabled, consider enabling it by adding: "
                       "\nextra_arguments=[{'use_intra_process_comms' : True}]\nto the Servo composable node "
                       "in the launch file");
  }
  servo_node_ = std::make_shared<rclcpp::Node>("servo_node", options);

  RCLCPP_INFO(get_logger(), "Creating servo...");
  // Get the servo parameters
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(servo_node_);
  if (servo_parameters == nullptr)
  {
    RCLCPP_ERROR(get_logger(), "Failed to load the servo parameters.");
    throw std::runtime_error("Failed to load the servo parameters.");
  }

  // Set up planning_scene_monitor
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      servo_node_, "robot_description", "planning_scene_monitor");
  planning_scene_monitor_->startStateMonitor(servo_parameters->joint_topic);
  planning_scene_monitor_->startSceneMonitor(servo_parameters->monitored_planning_scene_topic);
  planning_scene_monitor_->setPlanningScenePublishingFrequency(30);
  planning_scene_monitor_->getStateMonitor()->enableCopyDynamics(true);
  planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                        std::string(servo_node_->get_fully_qualified_name()) +
                                                            "/publish_planning_scene");

  // If the planning scene monitor in servo is the primary one we provide /get_planning_scene service so RViz displays
  // or secondary planning scene monitors can fetch the scene, otherwise we request the planning scene from the
  // primary planning scene monitor (e.g. move_group)
  if (servo_parameters->is_primary_planning_scene_monitor)
    planning_scene_monitor_->providePlanningSceneService();
  else
    planning_scene_monitor_->requestPlanningSceneState();

  // Create servo
  RCLCPP_INFO(get_logger(), "Creating servo...");
  servo_ = std::make_unique<moveit_servo::Servo>(servo_node_, servo_parameters, planning_scene_monitor_);

  // Create pose tracker
  RCLCPP_INFO(get_logger(), "Creating pose tracker...");
  pose_tracker_ = std::make_unique<moveit_servo::PoseTracking>(servo_node_, servo_parameters, planning_scene_monitor_);

  // Create service
  RCLCPP_INFO(get_logger(), "Starting state change services...");
  rt_cmd_service_ = create_service<arm_interfaces::srv::RTCommand>(
      "/cmd_rt_mode", [this](const std::shared_ptr<RTRequest> request, const std::shared_ptr<RTResponse> response) {
        handleTransitionRequests(request, response);
      });

  // Create command subscribers
  RCLCPP_INFO(get_logger(), "Starting subscriptions...");
  joint_jog_subscriber_ = create_subscription<control_msgs::msg::JointJog>(
      "cmd_joint_jog", rclcpp::SystemDefaultsQoS(),
      [this](std::shared_ptr<control_msgs::msg::JointJog> cmd) { processJointJogCommand(cmd); });
  cartician_velocity_subscriber_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_cartician_velocity", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> cmd) { processTwistCommand(cmd); });
  rt_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "cmd_rt_pose", rclcpp::SystemDefaultsQoS(),
      [this](std::shared_ptr<geometry_msgs::msg::PoseStamped> cmd) { processPoseCommand(cmd); });

  // Create state publisher
  RCLCPP_INFO(get_logger(), "Starting publisher of the current state...");
  rt_status = create_publisher<std_msgs::msg::String>("arm_rt_server_state", rclcpp::SystemDefaultsQoS());
  std::thread([this]() {
    for (;;)
    {
      auto msg = std_msgs::msg::String();
      msg.data = std::to_string(state);
      rt_status->publish(msg);
      rclcpp::Rate(1).sleep();
    }
  }).detach();
  std::thread([this]() { this->loop(); }).detach();

  RCLCPP_INFO(get_logger(), "Arm RT Server initialization complete.");
}

ArmRTServer::~ArmRTServer() = default;

void ArmRTServer::processJointJogCommand(std::shared_ptr<control_msgs::msg::JointJog> joint_jog)
{
  if (mode != RT_MODE::JOG)
  {
    return;
  }

  latest_joint_jog_command = joint_jog;
  *recent_joint_jog_updated_ = true;
}

void ArmRTServer::processTwistCommand(std::shared_ptr<geometry_msgs::msg::TwistStamped> twist_command)
{
  if (mode != RT_MODE::TWIST)
  {
    return;
  }

  latest_twist_command = twist_command;
  *recent_twist_updated_ = true;
}

void ArmRTServer::processPoseCommand(std::shared_ptr<geometry_msgs::msg::PoseStamped> pose_command)
{
  if (mode != RT_MODE::POSE)
  {
    return;
  }

  latest_pose_command = pose_command;
  *recent_pose_updated_ = true;
}

void ArmRTServer::handleTransitionRequests(std::shared_ptr<RTRequest> request, std::shared_ptr<RTResponse> response)
{
  RCLCPP_INFO(get_logger(),
              "Received mode change request: from current mode %s to mode %d, with action request to mode %d.",
              std::to_string(state).c_str(), request->mode, request->action);

  switch (request->mode)
  {
    case 0:  // stop
      RCLCPP_INFO(get_logger(), "Transition to STOP...");
      stop();
      response->success = true;
      break;
    case 1:  // halt
      RCLCPP_INFO(get_logger(), "Transition to HALT...");
      halt();
      mode = RT_MODE_MAP.at(request->action);
      state = CONTROL_STATUS::HALT;
      response->success = true;
      break;
    case 2:  // active
      RCLCPP_INFO(get_logger(), "Transition to ACTIVE...");
      mode = RT_MODE_MAP.at(request->action);
      if (!safety_lock_on)
      {
        active();
        state = CONTROL_STATUS::ACTIVE;
        response->success = true;
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Safety lock on. Unable to be active. Please first go to HALT state to unlock.");
        response->success = false;
      }

      break;
    default:
      response->success = false;
      RCLCPP_INFO(get_logger(), "Invalid mode choice! Mode should be [0, 2], but receiving mode %d.", request->mode);
  }
}

void ArmRTServer::active()
{
  servo_->setPaused(false);
  pose_tracker_->stopMotion();
  safety_lock_on = true;
}

void ArmRTServer::halt()
{
  servo_->setPaused(true);
  pose_tracker_->stopMotion();
  safety_lock_on = false;
}

void ArmRTServer::stop()
{
  servo_->setPaused(true);
  pose_tracker_->stopMotion();
  pose_tracker_->resetTargetPose();
  safety_lock_on = true;
}

void ArmRTServer::loop()
{
  servo_->setPaused(false);
  // switch (state)
  // {
  //   case OFF:
  //   case HALT:
  //     // reset all the flags so none of the command send on these states actually is effective
  //     *recent_joint_jog_updated_ = false;
  //     *recent_twist_updated_ = false;
  //     *recent_pose_updated_ = false;
  //     break;
  //   case ACTIVE:
  //     // not setting the flags anymore. the new requests from the client will reset the flag and make the commands
  //     // effective.
  //     switch (mode)
  //     {
  //       case TWIST:
  //         if (latest_twist_command)
  //         {
  //         }
  //         break;
  //       case POSE:
  //         if (recent_pose_updated_)
  //         {
  //           pose_tracker_->moveToPose(Eigen::Vector3d::Ones() * 0.02, 3.0 / 360 * 2 * 3.1415926, 10.0);
  //           *recent_pose_updated_ = false;
  //         }
  //         break;
  //       case JOG:
  //         if (recent_joint_jog_updated_)
  //         {
  //         }
  //         break;
  //     }
  //     break;
  // }
  servo_->start();
}

}  // namespace urc_arm::server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(urc_arm::server::ArmRTServer);
