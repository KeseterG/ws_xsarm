#include "arm_test/arm_server.hpp"
#include <algorithm>
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <limits>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <thread>

namespace urc_arm::server
{
ArmPlanAndExecuteServer::ArmPlanAndExecuteServer(const rclcpp::NodeOptions& options)
  : rclcpp::Node("arm_planning_server", options)
{
  // output
  RCLCPP_INFO(this->get_logger(), "Initializing arm planning server...");

  // parameters
  declare_parameter<double>("x_positive_limit", std::numeric_limits<double>::infinity());
  declare_parameter<double>("y_positive_limit", std::numeric_limits<double>::infinity());
  declare_parameter<double>("z_positive_limit", std::numeric_limits<double>::infinity());
  declare_parameter<double>("x_negative_limit", -std::numeric_limits<double>::infinity());
  declare_parameter<double>("y_negative_limit", -std::numeric_limits<double>::infinity());
  declare_parameter<double>("z_negative_limit", -std::numeric_limits<double>::infinity());

  declare_parameter<std::string>("arm_planning_group", "interbotix_arm");

  // spin the node for getting parameters
  move_group_node_ = rclcpp::Node::make_shared("move_group_node");
  move_group_node_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  move_group_node_executor->add_node(move_group_node_);
  std::thread([this]() { this->move_group_node_executor->spin(); }).detach();

  // // setup move groups & joint model group
  std::string arm_planning_group_name = get_parameter("arm_planning_group").as_string();
  move_group_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, arm_planning_group_name);
  RCLCPP_INFO(this->get_logger(), "Succesfully lode move group %s for the arm.", arm_planning_group_name.c_str());

  // start action server
  RCLCPP_INFO(this->get_logger(), "Starting action server...");
  using namespace std::placeholders;

  this->pose_planning_action_server_ = rclcpp_action::create_server<Plan>(
      this, "arm_planning_action_server", std::bind(&ArmPlanAndExecuteServer::handle_plan_goal, this, _1, _2),
      std::bind(&ArmPlanAndExecuteServer::handle_plan_cancel, this, _1),
      std::bind(&ArmPlanAndExecuteServer::handle_plan_accepted, this, _1));
  RCLCPP_INFO(this->get_logger(), "Planning action server started successfully.");

  this->executing_server_ = rclcpp_action::create_server<Execute>(
      this, "arm_executing_action_server", std::bind(&ArmPlanAndExecuteServer::handle_execute_goal, this, _1, _2),
      std::bind(&ArmPlanAndExecuteServer::handle_execute_cancel, this, _1),
      std::bind(&ArmPlanAndExecuteServer::handle_execute_accepted, this, _1));
  RCLCPP_INFO(this->get_logger(), "Executing action server started successfully.");
}

rclcpp_action::GoalResponse ArmPlanAndExecuteServer::handle_plan_goal(const rclcpp_action::GoalUUID& uuid,
                                                                      std::shared_ptr<const PlanGoal> goal)
{
  RCLCPP_INFO(get_logger(), "Received %s Goal.", goal->mode == 0 ? "Pose Planning" : "Position Planning");
  auto position = goal->target.position;
  if (position.x > this->get_parameter("x_positive_limit").as_double() ||
      position.y > this->get_parameter("y_positive_limit").as_double() ||
      position.z > this->get_parameter("z_positive_limit").as_double() ||
      position.x < this->get_parameter("x_negative_limit").as_double() ||
      position.y < this->get_parameter("y_negative_limit").as_double() ||
      position.z < this->get_parameter("z_negative_limit").as_double())
  {
    RCLCPP_INFO(get_logger(), "Goal out of predefined workspace. Reject goal request.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "Goal in predefined workspace. Accept goal request.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ArmPlanAndExecuteServer::handle_plan_cancel(const std::shared_ptr<PlanGoalHandle>& goal_handle)
{
  RCLCPP_INFO(get_logger(), "Cancel goal request %s.", goal_handle->get_goal_id().data());
  return rclcpp_action::CancelResponse::REJECT;
}

void ArmPlanAndExecuteServer::handle_plan_accepted(const std::shared_ptr<PlanGoalHandle>& goal_handle)
{
  auto current_goal_position = goal_handle->get_goal()->target.position;
  auto current_goal_orientation = goal_handle->get_goal()->target.orientation;
  RCLCPP_INFO(get_logger(),
              "Planning request accepted! Planning to pose <x: %.2f, y: %.2f, z: %.2f>, <x: %.2f, y: %.2f, z: %.2f, w: "
              "%.2f>, %s goals waiting ahead.",
              current_goal_position.x, current_goal_position.y, current_goal_position.z, current_goal_orientation.w,
              current_goal_orientation.x, current_goal_orientation.y, current_goal_orientation.z,
              is_planning_ ? "Has" : "No");
  std::thread([this, goal_handle]() { this->execute_pose_planning(goal_handle); }).detach();
}

void ArmPlanAndExecuteServer::execute_pose_planning(const std::shared_ptr<PlanGoalHandle>& goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing goal.");
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<PlanResult>();
  while (is_planning_)
  {
    if (goal_handle->is_canceling())
    {
      RCLCPP_INFO(get_logger(), "Goal %s cancelled.", goal_handle->get_goal_id().data());
      result->success = false;
      goal_handle->canceled(result);
      return;
    }
  }

  is_planning_ = true;  // lock
  RCLCPP_INFO(get_logger(), "Start planning goal.");
  move_group_->setPoseTarget(goal->target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = move_group_->plan(plan);

  if (plan_result == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(get_logger(), "Planning success!");
    result->trajectory = plan.trajectory_;
    result->planning_time = plan.planning_time_;
    result->start_state = plan.start_state_;
    result->success = true;
    move_group_->execute(plan);
    goal_handle->succeed(result);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Planning failed!");
    result->success = false;
    goal_handle->abort(result);
  }
  is_planning_ = false;  // unlock
}

rclcpp_action::GoalResponse ArmPlanAndExecuteServer::handle_execute_goal(const rclcpp_action::GoalUUID& uuid,
                                                                         std::shared_ptr<const ExecuteGoal> goal)
{
  RCLCPP_INFO(get_logger(), "Request execute request. %s override the current trajectory.",
              goal->override ? "Will" : "Will not");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ArmPlanAndExecuteServer::handle_execute_cancel(const std::shared_ptr<ExecuteGoalHandle>& goal_handle)
{
  RCLCPP_INFO(get_logger(), "Cancel executing request.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmPlanAndExecuteServer::handle_execute_accepted(const std::shared_ptr<ExecuteGoalHandle>& goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing requset accecpted.");
  std::thread([this, goal_handle]() { exeucute_arm_movement(goal_handle); }).detach();
}

void ArmPlanAndExecuteServer::exeucute_arm_movement(const std::shared_ptr<ExecuteGoalHandle>& goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ExecuteResult>();
  if (goal->override)
  {
    move_group_->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    is_planning_ = false;
  }

  while (is_executing_ && !goal->override)
  {
    if (goal_handle->is_canceling())
    {
      RCLCPP_INFO(get_logger(), "Goal cancelled.");
      result->success = false;
      goal_handle->canceled(result);
      return;
    }
  }

  is_executing_ = true;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = goal->trajectory;
  plan.start_state_ = goal->start_state;
  moveit::core::MoveItErrorCode execute_result;

  is_executing_on_movement_ = true;  // flag for status feedback
  std::thread([this, goal_handle]() {
    RCLCPP_INFO(get_logger(), "Starting feedback!");
    std::shared_ptr<ExecuteFeedback> feedback;
    while (is_executing_on_movement_)
    {
      feedback->current_pose = move_group_->getCurrentPose().pose;
      goal_handle->publish_feedback(feedback);
    }
    RCLCPP_INFO(get_logger(), "Feedback done!");
  }).detach();
  execute_result = move_group_->execute(plan);
  is_executing_on_movement_ = false;  // stop feedback thread

  if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(get_logger(), "Executing success!");
    result->final_pose = move_group_->getCurrentPose().pose;
    result->success = true;
    move_group_->execute(plan);
    goal_handle->succeed(result);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Executing failed!");
    result->success = false;
    goal_handle->abort(result);
  }
  is_planning_ = false;  // unlock
}

ArmPlanAndExecuteServer::~ArmPlanAndExecuteServer() = default;
}  // namespace urc_arm::server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(urc_arm::server::ArmPlanAndExecuteServer);
