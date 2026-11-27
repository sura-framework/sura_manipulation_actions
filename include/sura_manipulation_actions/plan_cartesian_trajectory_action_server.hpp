#pragma once

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sura_manipulation_actions/action/plan_cartesian_trajectory.hpp"
#include "sura_manipulation_actions/action/follow_cartesian_trajectory.hpp"
#include "sura_manipulation_actions/action/check_cartesian_trajectory.hpp"
#include "sura_manipulation_actions/action/prepare_cartesian_posture.hpp"
#include "sura_manipulation_actions/single_arm_cartesian_planner.hpp"
#include "sura_manipulation_actions/cartesian_moveit_context.hpp"
#include "sura_manipulation_actions/cartesian_trajectory_visualization.hpp"
#include "sura_manipulation_actions/trajectory_execution.hpp"
#include "sura_manipulation_actions/cartesian_pose_path.hpp"
#include "sura_manipulation_actions/whole_body_nominal.hpp"
#include "sura_manipulation_actions/msg/robot_nominal_reference.hpp"

namespace sura_manipulation_actions
{

class PlanCartesianTrajectoryActionServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using Action = sura_manipulation_actions::action::PlanCartesianTrajectory;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using FollowAction = sura_manipulation_actions::action::FollowCartesianTrajectory;
  using FollowHandle = rclcpp_action::ClientGoalHandle<FollowAction>;
  using PrepareAction = sura_manipulation_actions::action::PrepareCartesianPosture;
  using CheckAction = sura_manipulation_actions::action::CheckCartesianTrajectory;
  using CheckHandle = rclcpp_action::ClientGoalHandle<CheckAction>;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  explicit PlanCartesianTrajectoryActionServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlanCartesianTrajectoryActionServer() override;

private:
  enum class Stage {Idle, Planning, Pending, Executing};
  struct TaskEndpoint
  {
    std::string action, controller_name, manager_service;
  };
  // Last offset (tip pose in the task frame) heard on the pose task's offset topic.
  struct OffsetState
  {
    std::mutex mutex;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    std::chrono::steady_clock::time_point received;
    bool valid = false;
  };
  struct PoseEndpoint
  {
    std::string controller_node, task_id, target_topic, feedforward_topic, check_action, prepare_action;
    std::string offset_topic;
    // Created once at configure: creating and destroying clients while the node spins races in Humble's executor.
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr parameters;
    std::shared_ptr<OffsetState> offset = std::make_shared<OffsetState>();
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr offset_subscription;
    std::string nominal_topic;
    rclcpp_action::Client<PrepareAction>::SharedPtr preparer;
    rclcpp_action::Client<CheckAction>::SharedPtr checker;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr feedforward;
    rclcpp::Publisher<msg::RobotNominalReference>::SharedPtr nominal;
  };
  struct PendingPlan
  {
    std::shared_ptr<Handle> handle;
    bimanual_cartesian::Arm arm;
    std::string backend;
    bimanual_cartesian::GoalSpec goal;
    bimanual_cartesian::Options options;
    bimanual_cartesian::PlannedTrajectory plan;
    CartesianViews views;
    CartesianPosePath pose_path;
    msg::PosturePreparationSummary proposal;
    bool posture_applied = false;
    // Whole-body nominal streamed to robot_nominal (optimize_trajectory_nominal).
    bool wants_nominal = false;
    WholeBodyNominal nominal;
    bool use_predictive_local_planner = false;
    std::vector<double> nominal_progress;
    trajectory_msgs::msg::JointTrajectory nominal_joints;
    trajectory_msgs::msg::MultiDOFJointTrajectory nominal_base;
    // The pose task controls the offset point (zip-tie tip): poses and target are tip poses.
    bool use_tip_offset = false;
    Eigen::Isometry3d pose_start = Eigen::Isometry3d::Identity();
    Eigen::Matrix<double, 6, 1> pose_activation = Eigen::Matrix<double, 6, 1>::Ones();
    Eigen::Isometry3d fixed_from_model = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d base_reference = Eigen::Isometry3d::Identity();
    sura_manipulation_actions::msg::BaseMotionLimits base_limits;
    sura_manipulation_actions::msg::TrajectoryTrackingTolerances tracking_tolerances;
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
  void executePlan(const std::shared_ptr<PendingPlan> & pending);
  void planPoseGoal(const std::shared_ptr<PendingPlan> & pending);
  bool executePredictive(const PendingPlan &, msg::RobotNominalReference &, std::string &);
  void executePosePlan(const std::shared_ptr<PendingPlan> & pending);
  bool poseBackendReady(PendingPlan & pending, bool & auto_update, std::string & error);
  bool preparePosture(PendingPlan & pending, bool apply, std::string & error);
  bool checkPoseTrajectory(const PendingPlan &, uint64_t & validation_id,
    bool & retryable, std::string & error);
  bool validatePosePending(const PendingPlan & pending, std::string & error) const;
  bool readTipPose(const PendingPlan & pending, Eigen::Isometry3d & pose, std::string & error) const;
  // Reads the pose task's use_offset parameter from the controller (false if it is not declared).
  bool readUseOffset(const PendingPlan & pending, bool & use, double timeout, std::string & error);
  // Latest fresh offset of the pose task; fails if none was received or it is older than pose.offset_max_age.
  bool readTipOffset(const PendingPlan & pending, Eigen::Isometry3d & offset, std::string & error) const;
  void executeCoordinated(const std::shared_ptr<PendingPlan> & pending);
  void cancelCoordinated();
  bool checkControllerTracking(
    const PendingPlan & pending, double elapsed, bool final, std::string & error) const;
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
    const std::string & stage, const std::string & message,
    const msg::PosturePreparationSummary * proposal = nullptr) const;
  void finish(const std::shared_ptr<PendingPlan> & pending, bool success, const std::string & message);
  void retain(const std::shared_ptr<PendingPlan> & pending, const std::string & message);
  void stop(const std::string & message);
  void cleanupResources();
  std::string topic(const std::string & suffix) const;

  std::string robot_namespace_, moveit_namespace_, action_name_, fixed_frame_, execution_backend_;
  std::string reference_frame_;
  std::string base_frame_, coordinated_action_, base_manager_service_, base_controller_name_;
  double state_timeout_{2.0}, state_max_age_{2.0}, planning_time_{5.0}, cartesian_step_{0.02};
  double start_joint_tolerance_{0.001}, start_prismatic_tolerance_{0.0001};
  double base_position_tolerance_{0.005}, base_orientation_tolerance_{0.01};
  double pose_linear_velocity_{1.0}, pose_angular_velocity_{0.5};
  double pose_linear_acceleration_{0.4}, pose_angular_acceleration_{0.2};
  bool predictive_enabled_{false}, require_tip_offset_{false};
  msg::LocalPrediction prediction_settings_;
  double prediction_period_{0.25};
  double stpik_timeout_{35.0};
  double pose_offset_max_age_{0.5};
  bimanual_cartesian::Options planner_options_;
  sura_manipulation_actions::msg::TrajectoryTrackingTolerances default_tracking_tolerances_;
  ExecutionOptions execution_options_;
  std::map<std::string, bimanual_cartesian::Arm> arms_;
  std::map<std::string, TaskEndpoint> tasks_;
  std::map<std::string, PoseEndpoint> pose_endpoints_;
  std::unique_ptr<tf2_ros::Buffer> pose_tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> pose_tf_listener_;
  std::unique_ptr<CartesianMoveItContext> context_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_base_path_pub_, predicted_path_pub_;
  std::unique_ptr<CartesianTrajectoryVisualization> visualization_;
  std::unique_ptr<TrajectoryExecutor> executor_;
  struct FollowDispatch
  {
    std::mutex mutex;
    bool canceled{false};
    FollowHandle::SharedPtr handle;
  };
  rclcpp_action::Client<FollowAction>::SharedPtr coordinated_client_;
  std::shared_ptr<FollowDispatch> follow_dispatch_;
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
