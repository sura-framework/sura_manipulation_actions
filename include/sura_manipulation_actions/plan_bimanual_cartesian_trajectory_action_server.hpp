#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sura_manipulation_actions/action/check_cartesian_trajectory.hpp"
#include "sura_manipulation_actions/action/prepare_cartesian_posture.hpp"
#include "sura_manipulation_actions/action/plan_bimanual_cartesian_trajectory.hpp"
#include "sura_manipulation_actions/bimanual_cartesian_planner.hpp"
#include "sura_manipulation_actions/cartesian_moveit_context.hpp"
#include "sura_manipulation_actions/cartesian_trajectory_visualization.hpp"
#include "sura_manipulation_actions/facing_messages.hpp"
#include "sura_manipulation_actions/srv/check_facing_execution.hpp"
#include "sura_manipulation_actions/trajectory_execution.hpp"
#include "sura_manipulation_actions/whole_body_nominal.hpp"

namespace sura_manipulation_actions
{

class PlanBimanualCartesianTrajectoryActionServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using Action = sura_manipulation_actions::action::PlanBimanualCartesianTrajectory;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  explicit PlanBimanualCartesianTrajectoryActionServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlanBimanualCartesianTrajectoryActionServer() override;

private:
  enum class Stage {Idle, Planning, Pending, Executing};
  // Facing backend: the task-priority facing task and robot_nominal as secondary task.
  struct FacingEndpoint
  {
    std::string facing_task, nominal_task, reference_topic, nominal_topic, check_action, ready_service;
    std::string prepare_action;
    rclcpp_action::Client<sura_manipulation_actions::action::PrepareCartesianPosture>::SharedPtr preparer;
    std::vector<std::string> disabled_tasks;
    rclcpp::Publisher<sura_manipulation_actions::msg::FacingReference>::SharedPtr reference;
    rclcpp::Publisher<sura_manipulation_actions::msg::RobotNominalReference>::SharedPtr nominal;
    rclcpp::Client<sura_manipulation_actions::srv::CheckFacingExecution>::SharedPtr ready;
    rclcpp_action::Client<sura_manipulation_actions::action::CheckCartesianTrajectory>::SharedPtr checker;
  };
  struct PendingPlan
  {
    std::shared_ptr<Handle> handle;
    bimanual_cartesian::Arm arm_a, arm_b;
    bimanual_cartesian::Pair pair;
    bimanual_cartesian::GoalSpec goal;
    bimanual_cartesian::Options options;
    std::string backend;  // controller, task or facing; resolved per goal
    bool use_stpik{true};
    // Whole trajectory optimized by the controller (robot_nominal knots, no quintic timing).
    bool optimized{false};
    trajectory_msgs::msg::MultiDOFJointTrajectory nominal_base;
    bimanual_cartesian::PlannedTrajectory plan;
    CartesianViews views;
    Eigen::Isometry3d fixed_from_model = Eigen::Isometry3d::Identity();
    std::atomic_bool finished{false};
  };

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  bool loadConfiguration(std::string & error);
  bool validateGoal(const Action::Goal & goal, PendingPlan & pending, std::string & error) const;
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<Handle> & handle);
  void handleAccepted(const std::shared_ptr<Handle> & handle);
  void planGoal(const std::shared_ptr<PendingPlan> & pending);
  bool planOptimized(
    const std::shared_ptr<PendingPlan> & pending, const planning_scene::PlanningScenePtr & scene,
    std::string & error);
  void executePlan(const std::shared_ptr<PendingPlan> & pending);
  void executeFacingPlan(const std::shared_ptr<PendingPlan> & pending);
  bool facingTasksReady(const PendingPlan & pending, std::string & error);
  // Planned joints plus the constant measured base, as streamed to robot_nominal.
  bool wholeBodyNominal(
    const PendingPlan & pending, trajectory_msgs::msg::JointTrajectory & joints,
    trajectory_msgs::msg::MultiDOFJointTrajectory & base, std::string & error) const;
  bool checkFacingStpik(
    const std::shared_ptr<PendingPlan> & pending, bool & retryable, std::string & error);
  bool backendReady(const PendingPlan & pending, std::string & error);
  bool validatePending(const PendingPlan & pending, std::string & error) const;
  void executeTrigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void cancelTrigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void feedback(
    const std::shared_ptr<Handle> & handle, float progress,
    const std::string & stage, const std::string & message) const;
  void finish(const std::shared_ptr<PendingPlan> & pending, bool success, const std::string & message);
  void retain(const std::shared_ptr<PendingPlan> & pending, const std::string & message);
  void stop(const std::string & message);
  void cleanupResources();
  std::string topic(const std::string & suffix) const;

  std::string robot_namespace_, moveit_namespace_, action_name_, fixed_frame_, execution_backend_;
  std::string relative_reference_frame_;
  double state_timeout_{2.0}, state_max_age_{2.0}, planning_time_{5.0}, cartesian_step_{0.005};
  double start_joint_tolerance_{0.001}, start_prismatic_tolerance_{0.0001};
  double base_position_tolerance_{0.005}, base_orientation_tolerance_{0.01};
  // STPIK tolerances per facing row: left pointing (2), right pointing (2), roll, distance.
  std::vector<double> facing_path_tolerance_{0.1, 0.1, 0.1, 0.1, 0.15, 0.02};
  std::vector<double> facing_goal_tolerance_{0.05, 0.05, 0.05, 0.05, 0.1, 0.01};
  double goal_time_tolerance_{0.5};
  // Optimized facing timing at velocity_scaling 1, and the controller planning budget.
  double facing_max_linear_velocity_{0.3}, facing_max_angular_velocity_{1.0};
  double facing_prepare_timeout_{120.0};
  double stpik_timeout_{35.0};
  std::string active_pair_;
  bimanual_cartesian::Options planner_options_;
  ExecutionOptions execution_options_;
  std::map<std::string, bimanual_cartesian::Arm> arms_;
  std::map<std::string, bimanual_cartesian::Pair> pairs_;
  FacingEndpoint facing_;
  std::unique_ptr<CartesianMoveItContext> context_;
  std::unique_ptr<CartesianTrajectoryVisualization> visualization_;
  std::unique_ptr<TrajectoryExecutor> executor_;
  rclcpp::Node::SharedPtr moveit_node_;
  rclcpp::CallbackGroup::SharedPtr clients_group_;
  rclcpp_action::Server<Action>::SharedPtr action_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_, cancel_service_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
  std::map<std::string, rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr> managers_;
  mutable std::mutex mutex_;
  Stage stage_{Stage::Idle};
  std::shared_ptr<PendingPlan> pending_;
  std::thread worker_;
  std::atomic_bool stopped_{false};
  std::atomic_bool execution_active_{false};
};

}  // namespace sura_manipulation_actions
