/**
 * @file arm_planning_server.hpp
 * @author Zimeng Chai (zimeng.chai@gatech.edu)
 * @brief Arm planning server for planning and executing arm movement.
 * @version 0.1
 * @date 2023-10-25
 *
 * @copyright Copyright (c) 2023
 *
 */

#if !defined(ARM_SERVER_HPP_)
#define ARM_SERVER_HPP_

#include <memory>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include "moveit/move_group_interface/move_group_interface.h"

#include "arm_interfaces/action/plan.hpp"
#include "arm_interfaces/action/execute.hpp"
#include "arm_interfaces/action/shutdown.hpp"
#include "arm_interfaces/action/enable.hpp"

namespace urc_arm::server
{

typedef arm_interfaces::action::Plan Plan;
typedef arm_interfaces::action::Execute Execute;

typedef rclcpp_action::Server<Plan> PlanServer;
typedef rclcpp_action::Server<Execute> ExecuteServer;

typedef Plan::Goal PlanGoal;
typedef Plan::Result PlanResult;
typedef rclcpp_action::ServerGoalHandle<Plan> PlanGoalHandle;
typedef Execute::Goal ExecuteGoal;
typedef Execute::Result ExecuteResult;
typedef Execute::Feedback ExecuteFeedback;
typedef rclcpp_action::ServerGoalHandle<Execute> ExecuteGoalHandle;

typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LifecycleCallback;

/**
 * @brief A central action server for all possible movement requests from the client.
 */
class ArmServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ArmServer(const rclcpp::NodeOptions& options);
  ~ArmServer();

private:
  // server definition
  std::shared_ptr<PlanServer> pose_planning_action_server_;
  std::shared_ptr<ExecuteServer> executing_server_;

  // server callback functions - planning
  rclcpp_action::GoalResponse handle_plan_goal(const rclcpp_action::GoalUUID& uuid,
                                               std::shared_ptr<const PlanGoal> goal);
  rclcpp_action::CancelResponse handle_plan_cancel(const std::shared_ptr<PlanGoalHandle>& goal_handle);
  void handle_plan_accepted(const std::shared_ptr<PlanGoalHandle>& goal_handle);
  void execute_pose_planning(const std::shared_ptr<PlanGoalHandle>& goal_handle);

  // server callback functions - executing
  rclcpp_action::GoalResponse handle_execute_goal(const rclcpp_action::GoalUUID& uuid,
                                                  std::shared_ptr<const ExecuteGoal> goal);
  rclcpp_action::CancelResponse handle_execute_cancel(const std::shared_ptr<ExecuteGoalHandle>& goal_handle);
  void handle_execute_accepted(const std::shared_ptr<ExecuteGoalHandle>& goal_handle);
  void exeucute_arm_movement(const std::shared_ptr<ExecuteGoalHandle>& goal_handle);

  // movegroup related
  rclcpp::Node::SharedPtr move_group_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> move_group_node_executor;

  // planning & executing status flag
  std::atomic_bool is_planning_ = false;
  std::atomic_bool is_executing_ = false;
  std::atomic_bool is_executing_on_movement_ = false;  // for status feedback

  // lifecycle methods
  LifecycleCallback on_configure(const rclcpp_lifecycle::State& previous_state) override;
  LifecycleCallback on_activate(const rclcpp_lifecycle::State& previous_state) override;
  LifecycleCallback on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  LifecycleCallback on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  LifecycleCallback on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
};

}  // namespace urc_arm::server

#endif  // ARM_SERVER_HPP_
