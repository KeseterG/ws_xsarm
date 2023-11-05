#ifndef ARM_RT_SERVER
#define ARM_RT_SERVER

#include <arm_interfaces/srv/detail/rt_command__struct.hpp>
#include <atomic>
#include <control_msgs/msg/detail/joint_jog__struct.hpp>
#include <cstdint>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <map>
#include <memory>
#include "map"
#include "thread"

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "moveit_servo/servo.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <moveit_msgs/msg/detail/collision_object__struct.hpp>
#include <moveit_msgs/msg/detail/planning_scene__struct.hpp>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_srvs/srv/detail/set_bool__struct.hpp>
#include <thread>

#include "arm_interfaces/srv/rt_command.hpp"

namespace urc_arm::server
{

typedef arm_interfaces::srv::RTCommand RTCommand;
typedef arm_interfaces::srv::RTCommand::Request RTRequest;
typedef arm_interfaces::srv::RTCommand::Response RTResponse;

enum CONTROL_STATUS
{
  OFF,
  HALT,
  ACTIVE
};

enum RT_MODE
{
  POSE,
  TWIST,
  JOG
};

const std::map<int8_t, RT_MODE> RT_MODE_MAP{ { 0, POSE }, { 1, TWIST }, { 2, JOG } };

class ArmRTServer : public rclcpp::Node
{
public:
  explicit ArmRTServer(const rclcpp::NodeOptions& options);
  ~ArmRTServer();

private:
  // servo executor and nodes
  std::shared_ptr<rclcpp::Node> servo_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> servo_node_executor_;
  std::unique_ptr<moveit_servo::Servo> servo_;
  std::unique_ptr<moveit_servo::PoseTracking> pose_tracker_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  std::shared_ptr<std::thread> servo_loop_thread_;

  // rt command state and stauts flags
  std::shared_ptr<geometry_msgs::msg::TwistStamped> latest_twist_command;
  std::shared_ptr<std::atomic_bool> recent_twist_updated_;
  std::shared_ptr<control_msgs::msg::JointJog> latest_joint_jog_command;
  std::shared_ptr<std::atomic_bool> recent_joint_jog_updated_;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> latest_pose_command;
  std::shared_ptr<std::atomic_bool> recent_pose_updated_;

  // mode & flags
  std::atomic<CONTROL_STATUS> state = CONTROL_STATUS::OFF;
  std::atomic<RT_MODE> mode = RT_MODE::TWIST;
  std::atomic_bool safety_lock_on = false;

  // command subscribers
  std::shared_ptr<rclcpp::Subscription<control_msgs::msg::JointJog>> joint_jog_subscriber_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>> cartician_velocity_subscriber_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> rt_pose_subscriber_;

  // status publisher
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> rt_status;

  // services provided
  std::shared_ptr<rclcpp::Service<RTCommand>> rt_cmd_service_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> pause_servo_service_;

  // methods for state transitions
  void handleTransitionRequests(std::shared_ptr<RTRequest> request, std::shared_ptr<RTResponse> response);

  void active();
  void halt();
  void stop();
  void processJointJogCommand(std::shared_ptr<control_msgs::msg::JointJog> joint_jog_cmd);
  void processTwistCommand(std::shared_ptr<geometry_msgs::msg::TwistStamped> twist_cmd);
  void processPoseCommand(std::shared_ptr<geometry_msgs::msg::PoseStamped> pose_cmd);

  void loop();
};

}  // namespace urc_arm::server

#endif  // !ARM_RT_SERVER
