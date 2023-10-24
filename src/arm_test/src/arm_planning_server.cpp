#include "arm_test/arm_planning_server.hpp"
#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>

namespace urc_arm::server
{
ArmPlanningServer::ArmPlanningServer(const rclcpp::NodeOptions& options) : rclcpp::Node("arm_planning_server", options)
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
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_group_node_);
  std::thread([&executor]() { executor.spin(); }).detach();

  // setup move groups & joint model group
  std::string arm_planning_group_name = get_parameter("arm_planning_group").as_string();
  move_group_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, arm_planning_group_name);
  RCLCPP_INFO(this->get_logger(), "Succesfully lode move group %s for the arm.", arm_planning_group_name.c_str());

  // // start action server
  RCLCPP_INFO(this->get_logger(), "Starting action server...");
  using namespace std::placeholders;

  this->pose_planning_action_server_ = rclcpp_action::create_server<Plan>(
      this, "arm_planning_action_server", std::bind(&ArmPlanningServer::handle_plan_goal, this, _1, _2),
      std::bind(&ArmPlanningServer::handle_plan_cancel, this, _1),
      std::bind(&ArmPlanningServer::handle_plan_accepted, this, _1));
  RCLCPP_INFO(this->get_logger(), "Planning action server started successfully.");
}

rclcpp_action::GoalResponse ArmPlanningServer::handle_plan_goal(const rclcpp_action::GoalUUID& uuid,
                                                                std::shared_ptr<const PlanGoal> goal)
{
  RCLCPP_INFO(get_logger(), "Received %s Goal.", goal->mode == 0 ? "Pose Planning" : "Position Planning");
  // simple goal filtering by ee workspace
  if ((goal->target.position.x > get_parameter("x_positive_limit").as_double() ||
       goal->target.position.y > get_parameter("y_positive_limit").as_double() ||
       goal->target.position.z > get_parameter("z_positive_limit").as_double() ||
       goal->target.position.x > get_parameter("x_negative_limit").as_double() ||
       goal->target.position.y > get_parameter("y_negative_limit").as_double() ||
       goal->target.position.z > get_parameter("z_negative_limit").as_double()))
  {
    RCLCPP_ERROR(get_logger(), "Arm planning position out of bounds.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (is_planning_)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  else
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
  }
}

rclcpp_action::CancelResponse ArmPlanningServer::handle_plan_cancel(const std::shared_ptr<PlanGoalHandle>& goal_handle)
{
  RCLCPP_INFO(get_logger(), "Cancel goal request %s.", goal_handle->get_goal_id().data());
  return rclcpp_action::CancelResponse::REJECT;
}

void ArmPlanningServer::handle_plan_accepted(const std::shared_ptr<PlanGoalHandle>& goal_handle)
{
  auto current_goal_position = goal_handle->get_goal()->target.position;
  auto current_goal_orientation = goal_handle->get_goal()->target.orientation;
  RCLCPP_INFO(get_logger(),
              "Planning request accepted! Planning to pose <x: %.2f, y: %.2f, z: %.2f>, <x: %.2f, y: %.2f, z: %.2f, w: "
              "%.2f>, %s goals waiting ahead.",
              current_goal_position.x, current_goal_position.y, current_goal_position.z, current_goal_orientation.w,
              current_goal_orientation.x, current_goal_orientation.y, current_goal_orientation.z,
              is_planning_ ? "Has" : "No");
  std::thread([this, goal_handle]() { this->execute(goal_handle); }).detach();
}

void ArmPlanningServer::execute(const std::shared_ptr<PlanGoalHandle>& goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing goal %s.", goal_handle->get_goal_id().data());
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

  is_planning_ = true;
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

ArmPlanningServer::~ArmPlanningServer() = default;
}  // namespace urc_arm::server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(urc_arm::server::ArmPlanningServer);
