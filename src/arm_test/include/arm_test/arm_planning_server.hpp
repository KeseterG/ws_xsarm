
#if !defined(ARM_SERVER_HPP_)
#define ARM_SERVER_HPP_

#include <memory>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
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

typedef rclcpp_action::Server<Plan> ActionServer;
typedef rclcpp_action::Server<Execute> ExecuteServer;

typedef Plan::Goal PlanGoal;
typedef Plan::Result PlanResult;
typedef rclcpp_action::ServerGoalHandle<Plan> PlanGoalHandle;
typedef Execute::Goal ExecuteGoal;
typedef Execute::Result ExecuteResult;
typedef rclcpp_action::ServerGoalHandle<Execute> ExecuteGoalHandle;

/**
 * @brief A central action server for all the incoming requests from the BT action nodes for the arm.
 * The arm uses behavior tree architecture.
 */
class ArmPlanningServer : public rclcpp::Node
{
public:
  explicit ArmPlanningServer(const rclcpp::NodeOptions& options);
  ~ArmPlanningServer();

private:
  // server definition
  std::shared_ptr<ActionServer> pose_planning_action_server_;

  // server callback functions - planning
  rclcpp_action::GoalResponse handle_plan_goal(const rclcpp_action::GoalUUID& uuid,
                                               std::shared_ptr<const PlanGoal> goal);
  rclcpp_action::CancelResponse handle_plan_cancel(const std::shared_ptr<PlanGoalHandle>& goal_handle);
  void handle_plan_accepted(const std::shared_ptr<PlanGoalHandle>& goal_handle);

  // server callback functions - executing
  rclcpp_action::GoalResponse handle_execute_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<PlanGoal> goal);
  rclcpp_action::CancelResponse handle_execute_cancel(const std::shared_ptr<PlanGoalHandle>& goal_handle);
  void handle_execute_accepted(const std::shared_ptr<PlanGoalHandle>& goal_handle);
  void execute(const std::shared_ptr<PlanGoalHandle>& goal_handle);

  // movegroup related
  rclcpp::Node::SharedPtr move_group_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  // planning status flag
  std::atomic_bool is_planning_;
};

}  // namespace urc_arm::server

#endif  // ARM_SERVER_HPP_
