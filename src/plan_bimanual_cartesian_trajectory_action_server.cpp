#include "sura_manipulation_actions/plan_bimanual_cartesian_trajectory_action_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <set>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <sstream>

#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit/robot_model/revolute_joint_model.h"
#include "moveit/robot_state/robot_state.h"

namespace sura_manipulation_actions
{
namespace
{
using CheckAction = sura_manipulation_actions::action::CheckCartesianTrajectory;
template<typename T>
T parameter(rclcpp_lifecycle::LifecycleNode & node, const std::string & name, const T & fallback)
{
  if (node.has_parameter(name)) {
    const auto parameter = node.get_parameter(name);
    if constexpr (std::is_same_v<T, std::vector<std::string>> ||
      std::is_same_v<T, std::vector<double>>)
    {
      // ROS/launch can infer a different array type for an untyped YAML [].
      // An empty array still means no configured names/limits/nominal entries.
      const auto value = parameter.get_parameter_value().to_value_msg();
      if (parameter.get_type() >= rclcpp::ParameterType::PARAMETER_BYTE_ARRAY &&
        value.byte_array_value.empty() && value.bool_array_value.empty() &&
        value.integer_array_value.empty() && value.double_array_value.empty() &&
        value.string_array_value.empty()) {return T{};}
    }
    return parameter.get_value<T>();
  }
  return node.declare_parameter<T>(name, fallback);
}

Eigen::Vector3d vector(const geometry_msgs::msg::Vector3 & msg)
{
  return {msg.x, msg.y, msg.z};
}

std::vector<std::string> activeVariables(const moveit::core::JointModelGroup * group)
{
  std::vector<std::string> names;
  for (const auto * joint : group->getActiveJointModels()) {
    const auto & variables = joint->getVariableNames();
    names.insert(names.end(), variables.begin(), variables.end());
  }
  return names;
}
}  // namespace

PlanBimanualCartesianTrajectoryActionServer::PlanBimanualCartesianTrajectoryActionServer(
  const rclcpp::NodeOptions & options)
: LifecycleNode(
    "plan_bimanual_cartesian_trajectory_lifecycle_action_server",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)) {}

PlanBimanualCartesianTrajectoryActionServer::~PlanBimanualCartesianTrajectoryActionServer()
{
  cleanupResources();
}

std::string PlanBimanualCartesianTrajectoryActionServer::topic(const std::string & suffix) const
{
  std::string prefix = robot_namespace_;
  while (!prefix.empty() && prefix.front() == '/') {prefix.erase(0, 1);}
  while (!prefix.empty() && prefix.back() == '/') {prefix.pop_back();}
  return (prefix.empty() ? std::string() : "/" + prefix) + "/" + suffix;
}

bool PlanBimanualCartesianTrajectoryActionServer::loadConfiguration(std::string & error)
{
  robot_namespace_ = parameter<std::string>(*this, "robot_namespace", "cirtesub");
  moveit_namespace_ = parameter<std::string>(*this, "moveit_namespace", "moveit");
  action_name_ = parameter<std::string>(*this, "action_name", topic("manipulation/plan_bimanual_cartesian_trajectory"));
  execution_backend_ = parameter<std::string>(*this, "execution_backend", "controller");
  if (execution_backend_ != "controller" && execution_backend_ != "task" && execution_backend_ != "pose") {
    error = "execution_backend must be controller, task or pose"; return false;
  }
  fixed_frame_ = parameter<std::string>(*this, "visualization.fixed_frame", "");
  relative_reference_frame_ = parameter<std::string>(*this, "relative_reference_frame", "");
  planning_time_ = parameter<double>(*this, "planning.default_planning_time", 5.0);
  cartesian_step_ = parameter<double>(*this, "planning.cartesian_step", 0.005);
  state_timeout_ = parameter<double>(*this, "current_state_timeout", 2.0);
  state_max_age_ = parameter<double>(*this, "current_state_max_age", 2.0);
  start_joint_tolerance_ = parameter<double>(*this, "execution.start_joint_tolerance", 0.001);
  start_prismatic_tolerance_ = parameter<double>(*this, "execution.start_prismatic_tolerance", 0.0001);
  base_position_tolerance_ = parameter<double>(*this, "execution.base_position_tolerance", 0.005);
  base_orientation_tolerance_ = parameter<double>(*this, "execution.base_orientation_tolerance", 0.01);
  const std::vector<double> positive{planning_time_, cartesian_step_, state_timeout_, state_max_age_,
    start_joint_tolerance_, start_prismatic_tolerance_, base_position_tolerance_, base_orientation_tolerance_};
  if (std::any_of(positive.begin(), positive.end(), [](double value) {return !std::isfinite(value) || value <= 0.0;})) {
    error = "Planning, state, and execution tolerance defaults must be finite and positive"; return false;
  }
  auto & p = planner_options_;
  p.max_iterations = static_cast<int>(parameter<int64_t>(*this, "planning.max_iterations", 100));
  p.max_waypoints = static_cast<int>(parameter<int64_t>(*this, "planning.max_waypoints", 2000));
  p.max_subdivision_depth = static_cast<int>(parameter<int64_t>(*this, "planning.max_subdivision_depth", 6));
  p.recovery_seeds = static_cast<int>(parameter<int64_t>(*this, "planning.recovery_seeds", 8));
  p.smoothing_passes = static_cast<int>(parameter<int64_t>(*this, "planning.smoothing_passes", 2));
  p.max_timing_refinements = static_cast<int>(parameter<int64_t>(*this, "planning.max_timing_refinements", 6));
  p.angular_step = parameter<double>(*this, "planning.angular_step", 0.05);
  p.position_weight = parameter<double>(*this, "planning.position_weight", 10.0);
  p.orientation_weight = parameter<double>(*this, "planning.orientation_weight", 2.0);
  p.continuity_weight = parameter<double>(*this, "planning.continuity_weight", 5.0);
  p.weights = parameter<std::vector<double>>(*this, "planning.weights", {});
  for (const double weight : p.weights) {
    if (!std::isfinite(weight) || weight < 0.0) {
      error = "planning.weights must be finite and non-negative";
      return false;
    }
  }
  p.nominal_weight = parameter<double>(*this, "planning.nominal_weight", 0.1);
  p.smoothness_weight = parameter<double>(*this, "planning.smoothness_weight", 0.5);
  p.damping = parameter<double>(*this, "planning.damping", 0.001);
  p.max_joint_step = parameter<double>(*this, "planning.max_joint_step", 0.05);
  p.max_prismatic_step = parameter<double>(*this, "planning.max_prismatic_step", 0.005);
  p.validation_joint_step = parameter<double>(*this, "planning.validation_joint_step", 0.01);
  p.validation_prismatic_step = parameter<double>(*this, "planning.validation_prismatic_step", 0.001);
  p.validation_time_step = parameter<double>(*this, "planning.validation_time_step", 0.01);
  p.position_epsilon = parameter<double>(*this, "planning.position_epsilon", 1e-6);
  p.orientation_epsilon = parameter<double>(*this, "planning.orientation_epsilon", 1e-6);
  p.velocity_scaling = parameter<double>(*this, "planning.velocity_scaling", 0.1);
  p.acceleration_scaling = parameter<double>(*this, "planning.acceleration_scaling", 0.1);
  p.optimize_trajectory = parameter<bool>(*this, "planning.optimize_trajectory", true);
  const auto limit_names = parameter<std::vector<std::string>>(*this, "planning.limit_joint_names", {});
  const auto lower = parameter<std::vector<double>>(*this, "planning.lower_limits", {});
  const auto upper = parameter<std::vector<double>>(*this, "planning.upper_limits", {});
  if (limit_names.size() != lower.size() || lower.size() != upper.size()) {
    error = "Named tightened joint limits have inconsistent lengths"; return false;
  }
  for (std::size_t i = 0; i < limit_names.size(); ++i) {
    if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] > upper[i] ||
      !p.tightened_limits.emplace(limit_names[i], std::make_pair(lower[i], upper[i])).second)
    {error = "Invalid or duplicate tightened joint limit: " + limit_names[i]; return false;}
  }
  const auto nominal_names = parameter<std::vector<std::string>>(*this, "planning.nominal_joint_names", {});
  const auto nominal = parameter<std::vector<double>>(*this, "planning.nominal_positions", {});
  if (nominal_names.size() != nominal.size()) {error = "Named nominal posture lengths differ"; return false;}
  for (std::size_t i = 0; i < nominal.size(); ++i) {
    if (!std::isfinite(nominal[i]) || !p.nominal_positions.emplace(nominal_names[i], nominal[i]).second) {
      error = "Invalid or duplicate nominal joint: " + nominal_names[i]; return false;
    }
  }
  execution_options_.controller_wait_timeout = parameter<double>(*this, "execution.controller_wait_timeout", 5.0);
  execution_options_.acceptance_timeout = parameter<double>(*this, "execution.acceptance_timeout", 2.0);
  execution_options_.start_delay = parameter<double>(*this, "execution.start_delay", 3.0);
  execution_options_.execution_wait_margin = parameter<double>(*this, "execution.wait_margin", 5.0);
  execution_options_.feedback_rate = parameter<double>(*this, "execution.feedback_rate", 10.0);
  const auto & execution = execution_options_;
  if (!std::isfinite(execution.controller_wait_timeout) || execution.controller_wait_timeout <= 0.0 ||
    !std::isfinite(execution.acceptance_timeout) || execution.acceptance_timeout <= 0.0 ||
    !std::isfinite(execution.start_delay) || execution.start_delay <= execution.acceptance_timeout ||
    !std::isfinite(execution.execution_wait_margin) || execution.execution_wait_margin <= 0.0 ||
    !std::isfinite(execution.feedback_rate) || execution.feedback_rate <= 0.0)
  {error = "Invalid execution timing; start_delay must exceed acceptance_timeout"; return false;}
  const auto arm_names = parameter<std::vector<std::string>>(*this, "arm_names", {});
  for (const auto & name : arm_names) {
    const auto prefix = "arms." + name + ".";
    bimanual_cartesian::Arm arm;
    arm.name = name;
    arm.group_name = parameter<std::string>(*this, prefix + "group", name);
    arm.tip_frame = parameter<std::string>(*this, prefix + "tip_frame", "");
    arm.controller_action = parameter<std::string>(*this, prefix + "controller_action", "");
    arm.controller_name = parameter<std::string>(*this, prefix + "controller_name", "");
    arm.manager_service = parameter<std::string>(*this, prefix + "manager_service", "");
    if (name.empty() || !arms_.emplace(name, arm).second) {error = "Empty or duplicate arm name"; return false;}
  }
  const auto pair_names = parameter<std::vector<std::string>>(*this, "pair_names", {});
  std::set<std::set<std::string>> registered_pairs;
  for (const auto & name : pair_names) {
    const auto prefix = "pairs." + name + ".";
    bimanual_cartesian::Pair pair;
    pair.arm_names = parameter<std::vector<std::string>>(*this, prefix + "arms", {});
    pair.group_name = parameter<std::string>(*this, prefix + "group", "");
    pair.task_action = parameter<std::string>(*this, prefix + "task_action", "");
    pair.task_controller_name = parameter<std::string>(*this, prefix + "task_controller_name", "");
    pair.task_manager_service = parameter<std::string>(*this, prefix + "task_manager_service", "");
    if (pair.arm_names.size() != 2 || pair.arm_names[0] == pair.arm_names[1] ||
      !arms_.count(pair.arm_names[0]) || !arms_.count(pair.arm_names[1]) ||
      !registered_pairs.insert(std::set<std::string>(pair.arm_names.begin(), pair.arm_names.end())).second ||
      !pairs_.emplace(name, pair).second)
    {error = "Invalid or duplicate pair: " + name; return false;}
  }
  if (arms_.size() < 2 || pairs_.empty()) {error = "Configure arm_names and pair_names for at least two arms"; return false;}
  // Pose endpoints are always prepared so that a goal can select the pose backend.
  {
    tip_path_tolerance_ = parameter<std::vector<double>>(*this, "execution.tip_path_tolerance", tip_path_tolerance_);
    tip_goal_tolerance_ = parameter<std::vector<double>>(*this, "execution.tip_goal_tolerance", tip_goal_tolerance_);
    goal_time_tolerance_ = parameter<double>(*this, "execution.goal_time_tolerance", goal_time_tolerance_);
    stpik_timeout_ = parameter<double>(*this, "pose.stpik_timeout", stpik_timeout_);
    if (tip_path_tolerance_.size() != 6 || tip_goal_tolerance_.size() != 6 ||
      std::any_of(tip_path_tolerance_.begin(), tip_path_tolerance_.end(), [](double v) {return !std::isfinite(v) || v < 0.0;}) ||
      std::any_of(tip_goal_tolerance_.begin(), tip_goal_tolerance_.end(), [](double v) {return !std::isfinite(v) || v < 0.0;}) ||
      !std::isfinite(goal_time_tolerance_) || goal_time_tolerance_ < 0.0 ||
      !std::isfinite(stpik_timeout_) || stpik_timeout_ <= 0.0)
    {
      error = "Pose tolerances must contain six finite nonnegative values; stpik_timeout must be positive";
      return false;
    }
    // Each arm streams to its own task-priority pose task (left_pose / right_pose by default).
    for (const auto & entry : arms_) {
      const auto & arm = entry.second;
      const auto prefix = "arms." + arm.name + ".";
      PoseEndpoint pose;
      pose.arm_name = arm.name;
      const auto manager_suffix = arm.manager_service.rfind("/controller_manager/list_controllers");
      const auto controller_namespace = manager_suffix != std::string::npos ?
        arm.manager_service.substr(0, manager_suffix) : "/" + robot_namespace_ + "/controller";
      pose.task_id = parameter<std::string>(*this, prefix + "pose_task",
        arm.name.find("right") != std::string::npos ? "right_pose" :
        arm.name.find("left") != std::string::npos ? "left_pose" : arm.name + "_pose");
      pose.controller_node = parameter<std::string>(*this, prefix + "pose_controller_node",
        controller_namespace + "/task_priority_controller");
      pose.target_topic = parameter<std::string>(*this, prefix + "pose_target_topic",
        controller_namespace + "/task_priority/tasks/" + pose.task_id + "/target");
      const auto feedforward_suffix = std::string("/target");
      const auto default_feedforward = pose.target_topic.size() > feedforward_suffix.size() &&
        pose.target_topic.compare(pose.target_topic.size() - feedforward_suffix.size(),
        feedforward_suffix.size(), feedforward_suffix) == 0 ?
        pose.target_topic.substr(0, pose.target_topic.size() - feedforward_suffix.size()) + "/feedforward" :
        controller_namespace + "/task_priority/tasks/" + pose.task_id + "/feedforward";
      pose.feedforward_topic = parameter<std::string>(*this, prefix + "pose_feedforward_topic", default_feedforward);
      const auto tasks_separator = pose.target_topic.rfind("/tasks/");
      pose.check_action = parameter<std::string>(*this, prefix + "pose_stpik_action",
        (tasks_separator == std::string::npos ? controller_namespace + "/task_priority" :
        pose.target_topic.substr(0, tasks_separator)) + "/check_cartesian_trajectory");
      if (pose.task_id.empty() || pose.controller_node.empty() || pose.target_topic.empty() || pose.feedforward_topic.empty() ||
        pose.check_action.empty())
      {
        error = "Pose endpoint cannot be empty for arm: " + arm.name; return false;
      }
      pose_endpoints_.emplace(arm.name, std::move(pose));
    }
  }
  return true;
}

PlanBimanualCartesianTrajectoryActionServer::CallbackReturn
PlanBimanualCartesianTrajectoryActionServer::on_configure(const rclcpp_lifecycle::State &)
{
  cleanupResources();
  std::string error;
  try {
    if (!loadConfiguration(error)) {throw std::runtime_error(error);}
    const auto moveit_prefix = topic(moveit_namespace_);
    moveit_node_ = std::make_shared<rclcpp::Node>(
      "bimanual_cartesian_moveit_context", moveit_prefix,
      rclcpp::NodeOptions().allow_undeclared_parameters(true));
    for (const auto & name : list_parameters({}, 20).names) {
      const auto value = get_parameter(name);
      if (!moveit_node_->has_parameter(name)) {
        moveit_node_->declare_parameter(name, value.get_parameter_value());
      } else {moveit_node_->set_parameter(value);}
    }
    context_ = std::make_unique<CartesianMoveItContext>(moveit_node_);
    if (!context_->start(
        parameter<std::string>(*this, "joint_states_topic", topic("alpha/joint_states")),
        parameter<std::string>(*this, "planning_scene_input_topic", moveit_prefix + "/planning_scene"),
        parameter<std::string>(*this, "planning_scene_topic", moveit_prefix + "/monitored_planning_scene"),
        parameter<std::string>(*this, "display_trajectory_topic", moveit_prefix + "/display_planned_path"),
        parameter<double>(*this, "planning_scene_publish_rate", 10.0), error))
    {throw std::runtime_error(error);}
    const auto & model = context_->model();
    if (relative_reference_frame_.empty()) {relative_reference_frame_ = model->getModelFrame();}
    if (relative_reference_frame_ != model->getModelFrame() &&
      !model->hasLinkModel(relative_reference_frame_))
    {throw std::runtime_error("Unknown relative_reference_frame in robot model: " + relative_reference_frame_);}
    const auto & variables = model->getVariableNames();
    for (const auto & entry : planner_options_.tightened_limits) {
      if (std::find(variables.begin(), variables.end(), entry.first) == variables.end()) {
        throw std::runtime_error("Unknown joint variable in tightened limits: " + entry.first);
      }
    }
    for (const auto & entry : planner_options_.nominal_positions) {
      if (std::find(variables.begin(), variables.end(), entry.first) == variables.end()) {
        throw std::runtime_error("Unknown joint variable in nominal posture: " + entry.first);
      }
    }
    for (auto & entry : arms_) {
      auto & arm = entry.second;
      const auto * group = model->getJointModelGroup(arm.group_name);
      if (!group || !group->isChain() || !model->hasLinkModel(arm.tip_frame) ||
        !group->getUpdatedLinkModelsSet().count(model->getLinkModel(arm.tip_frame)))
      {throw std::runtime_error("Invalid serial group or tip for arm '" + arm.name + "'");}
      for (const auto * joint : group->getActiveJointModels()) {
        if (joint->getVariableCount() != 1) {throw std::runtime_error("Only scalar arm joints are supported");}
        if (joint->isPassive()) {throw std::runtime_error("Arm group contains a passive joint: " + joint->getName());}
      }
      arm.joint_names = activeVariables(group);
      if (arm.joint_names.empty()) {throw std::runtime_error("Arm group contains no active joints: " + arm.name);}
    }
    for (const auto & entry : pairs_) {
      const auto & pair = entry.second;
      std::set<std::string> expected;
      for (const auto & name : pair.arm_names) {
        for (const auto & variable : arms_.at(name).joint_names) {
          if (!expected.insert(variable).second) {throw std::runtime_error("Pair arms share an active joint");}
        }
      }
      const auto * group = model->getJointModelGroup(pair.group_name);
      if (!group) {throw std::runtime_error("Missing combined MoveIt group: " + pair.group_name);}
      if (relative_reference_frame_ != model->getModelFrame() &&
        group->getUpdatedLinkModelsSet().count(model->getLinkModel(relative_reference_frame_)))
      {throw std::runtime_error("relative_reference_frame must be independent of the commanded arms");}
      const auto names = activeVariables(group);
      if (expected != std::set<std::string>(names.begin(), names.end())) {
        throw std::runtime_error("Combined group must contain exactly the pair's active joints");
      }
      const auto & weights = planner_options_.weights;
      if (!weights.empty() && weights.size() != names.size()) {
        throw std::runtime_error("planning.weights must contain " + std::to_string(names.size()) +
          " entries for combined group '" + pair.group_name + "'");
      }
      for (std::size_t i = 0; i < names.size(); ++i) {
        RCLCPP_INFO(get_logger(), "planning.weights[%zu] (%s): %s = %.6g", i,
          pair.group_name.c_str(), names[i].c_str(), weights.empty() ? 0.0 : weights[i]);
      }
    }
    if (fixed_frame_.empty()) {fixed_frame_ = model->getModelFrame();}
    visualization_ = std::make_unique<CartesianTrajectoryVisualization>(
      moveit_node_, topic("bimanual_planner"), topic("manipulation"), fixed_frame_,
      parameter<bool>(*this, "visualization.publish_paths", true),
      parameter<bool>(*this, "visualization.publish_markers", true));
    for (auto & entry : pose_endpoints_) {
      entry.second.target = moveit_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        entry.second.target_topic, rclcpp::QoS(1).reliable());
      entry.second.feedforward = moveit_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        entry.second.feedforward_topic, rclcpp::QoS(1).reliable());
      entry.second.checker = rclcpp_action::create_client<CheckAction>(moveit_node_, entry.second.check_action);
    }
    executor_ = std::make_unique<TrajectoryExecutor>(this);
    clients_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    action_ = rclcpp_action::create_server<Action>(
      this, action_name_,
      std::bind(&PlanBimanualCartesianTrajectoryActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PlanBimanualCartesianTrajectoryActionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&PlanBimanualCartesianTrajectoryActionServer::handleAccepted, this, std::placeholders::_1));
    execute_service_ = create_service<std_srvs::srv::Trigger>(
      "~/execute_pending_trajectory",
      std::bind(&PlanBimanualCartesianTrajectoryActionServer::executeTrigger, this, std::placeholders::_1, std::placeholders::_2));
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "~/cancel_pending_trajectory",
      std::bind(&PlanBimanualCartesianTrajectoryActionServer::cancelTrigger, this, std::placeholders::_1, std::placeholders::_2));
    cancel_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
        std::shared_ptr<PendingPlan> pending;
        {std::lock_guard<std::mutex> lock(mutex_);
          if (stage_ == Stage::Pending && pending_ && pending_->handle->is_canceling()) {pending = pending_;}}
        if (pending) {finish(pending, false, "Pending plan canceled");}
      });
    stopped_.store(false);
    RCLCPP_INFO(get_logger(), "Configured '%s', execution backend '%s', manual activation", action_name_.c_str(), execution_backend_.c_str());
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Configuring Cartesian planner: %s", ex.what());
    cleanupResources();
    return CallbackReturn::FAILURE;
  }
}

PlanBimanualCartesianTrajectoryActionServer::CallbackReturn
PlanBimanualCartesianTrajectoryActionServer::on_activate(const rclcpp_lifecycle::State &)
{execution_active_.store(true); return CallbackReturn::SUCCESS;}

PlanBimanualCartesianTrajectoryActionServer::CallbackReturn
PlanBimanualCartesianTrajectoryActionServer::on_deactivate(const rclcpp_lifecycle::State &)
{execution_active_.store(false); stop("Cartesian planner deactivated"); return CallbackReturn::SUCCESS;}

PlanBimanualCartesianTrajectoryActionServer::CallbackReturn
PlanBimanualCartesianTrajectoryActionServer::on_cleanup(const rclcpp_lifecycle::State &)
{cleanupResources(); return CallbackReturn::SUCCESS;}

PlanBimanualCartesianTrajectoryActionServer::CallbackReturn
PlanBimanualCartesianTrajectoryActionServer::on_shutdown(const rclcpp_lifecycle::State &)
{cleanupResources(); return CallbackReturn::SUCCESS;}

bool PlanBimanualCartesianTrajectoryActionServer::validateGoal(
  const Action::Goal & goal, PendingPlan & pending, std::string & error) const
{
  const auto arm_a = arms_.find(goal.arm_a), arm_b = arms_.find(goal.arm_b);
  if (arm_a == arms_.end()) {error = "Unknown arm A: " + goal.arm_a; return false;}
  if (arm_b == arms_.end()) {error = "Unknown arm B: " + goal.arm_b; return false;}
  if (arm_a == arm_b) {error = "Arm A and arm B must be different arms"; return false;}
  if (goal.relative_target.header.frame_id != relative_reference_frame_) {
    error = "relative_target.frame_id must equal configured reference frame '" + relative_reference_frame_ + "'"; return false;
  }
  pending.backend = goal.execution_backend.empty() ? execution_backend_ : goal.execution_backend;
  pending.use_stpik = goal.use_stpik;
  if (pending.backend != "controller" && pending.backend != "task" && pending.backend != "pose") {
    error = "execution_backend must be empty, controller, task or pose"; return false;
  }
  pending.arm_a = arm_a->second; pending.arm_b = arm_b->second;
  bool found = false;
  for (const auto & entry : pairs_) {
    const auto & names = entry.second.arm_names;
    if (std::find(names.begin(), names.end(), goal.arm_a) != names.end() &&
      std::find(names.begin(), names.end(), goal.arm_b) != names.end())
    {pending.pair = entry.second; found = true; break;}
  }
  if (!found) {error = "The selected arm pair is not configured"; return false;}
  for (const auto scaling : {goal.velocity_scaling, goal.acceleration_scaling}) {
    if (!std::isfinite(scaling) || scaling < 0.0 || scaling > 1.0) {
      error = "velocity_scaling and acceleration_scaling must be finite in [0, 1]; zero uses server defaults";
      return false;
    }
  }
  // Capture the effective timing limits with the pending plan. Revalidation
  // must use these same per-goal limits when execution is requested later.
  pending.options = planner_options_;
  if (goal.velocity_scaling > 0.0) {pending.options.velocity_scaling = goal.velocity_scaling;}
  if (goal.acceleration_scaling > 0.0) {pending.options.acceleration_scaling = goal.acceleration_scaling;}
  auto & spec = pending.goal;
  spec.reference_frame = relative_reference_frame_;
  const auto & pose = goal.relative_target.pose;
  Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  spec.relative_target.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  spec.relative_position_tolerance = vector(goal.relative_position_tolerance);
  spec.relative_orientation_tolerance = vector(goal.relative_orientation_tolerance);
  if (!quaternion.coeffs().allFinite() || std::abs(quaternion.norm() - 1.0) > 0.001 ||
    !spec.relative_target.translation().allFinite() ||
    !spec.relative_position_tolerance.allFinite() || (spec.relative_position_tolerance.array() < 0.0).any() ||
    !spec.relative_orientation_tolerance.allFinite() || (spec.relative_orientation_tolerance.array() < 0.0).any() ||
    !std::isfinite(goal.cartesian_step) || !std::isfinite(goal.planning_time))
  {error = "Pose must have a unit quaternion and finite coordinates; constraints must be finite and non-negative"; return false;}
  spec.relative_target.linear() = quaternion.normalized().toRotationMatrix();
  spec.cartesian_step = goal.cartesian_step > 0.0 ? goal.cartesian_step : cartesian_step_;
  spec.planning_time = goal.planning_time > 0.0 ? goal.planning_time : planning_time_;
  return true;
}

rclcpp_action::GoalResponse PlanBimanualCartesianTrajectoryActionServer::handleGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stage_ == Stage::Planning || stage_ == Stage::Executing) {return rclcpp_action::GoalResponse::REJECT;}
  PendingPlan pending;
  std::string error;
  if (!validateGoal(*goal, pending, error)) {
    RCLCPP_WARN(get_logger(), "Rejected Cartesian goal: %s", error.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  stage_ = Stage::Planning;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlanBimanualCartesianTrajectoryActionServer::handleCancel(
  const std::shared_ptr<Handle> & handle)
{
  {std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_ || pending_->handle != handle) {return rclcpp_action::CancelResponse::REJECT;}}
  stopped_.store(true);
  executor_->cancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlanBimanualCartesianTrajectoryActionServer::handleAccepted(const std::shared_ptr<Handle> & handle)
{
  if (worker_.joinable()) {worker_.join();}
  auto pending = std::make_shared<PendingPlan>();
  std::string error;
  if (!validateGoal(*handle->get_goal(), *pending, error)) {
    pending->handle = handle;
    {std::lock_guard<std::mutex> lock(mutex_); stage_ = pending_ ? Stage::Pending : Stage::Idle;}
    finish(pending, false, error); return;
  }
  pending->handle = handle;
  std::shared_ptr<PendingPlan> previous;
  {std::lock_guard<std::mutex> lock(mutex_); previous = pending_; pending_ = pending; stage_ = Stage::Planning;}
  if (previous) {finish(previous, false, "Pending plan replaced by newer Cartesian goal");}
  stopped_.store(false);
  worker_ = std::thread([this, pending]() {planGoal(pending);});
}

void PlanBimanualCartesianTrajectoryActionServer::planGoal(const std::shared_ptr<PendingPlan> & pending)
{
  try {
    const auto canceled = [this, pending]() {return stopped_.load() || pending->handle->is_canceling() || !pending->handle->is_active();};
    std::vector<std::string> names = pending->arm_a.joint_names;
    names.insert(names.end(), pending->arm_b.joint_names.begin(), pending->arm_b.joint_names.end());
    planning_scene::PlanningScenePtr scene;
    std::string error;
    feedback(pending->handle, 0.0F, "acquiring_state", "Reading current joint states and planning scene");
    if (!context_->snapshot(names, state_timeout_, state_max_age_, scene, error, canceled) ||
      !visualization_->modelTransform(context_->model()->getModelFrame(), pending->fixed_from_model, error))
    {finish(pending, false, error); return;}
    const bimanual_cartesian::BimanualCartesianPlanner planner;
    if (!planner.plan(scene, scene->getCurrentState(), pending->arm_a, pending->arm_b, pending->pair,
        pending->goal, pending->options, pending->plan, error, canceled,
        [this, pending](double value, const std::string & stage) {
          feedback(pending->handle, static_cast<float>(0.1 + 0.7 * value), stage, "Planning constrained bimanual trajectory");
        }))
    {finish(pending, false, error); return;}
    if (canceled()) {finish(pending, false, "Planning canceled"); return;}
    pending->views = visualization_->publish(pending->plan, pending->arm_a, pending->arm_b);
    context_->display(*pending->plan.trajectory);
    retain(pending, "Trajectory planned and published; waiting for execute trigger");
    if (pending->handle->get_goal()->execute && !canceled()) {
      {std::lock_guard<std::mutex> lock(mutex_);
        if (pending_ != pending || stage_ != Stage::Pending) {return;}
        stage_ = Stage::Executing;}
      executePlan(pending);
    }
  } catch (const std::exception & ex) {finish(pending, false, std::string("Planning exception: ") + ex.what());}
}

bool PlanBimanualCartesianTrajectoryActionServer::backendReady(const PendingPlan & pending, std::string & error)
{
  if (!execution_active_.load()) {
    error = "Operator must activate the Cartesian planner lifecycle before execution";
    return false;
  }
  std::map<std::string, std::set<std::string>> required;
  if (pending.backend == "task" || pending.backend == "pose") {
    if (pending.backend == "task" && pending.pair.task_action.empty()) {
      error = "No joint task action configured"; return false;
    }
    if (!pending.pair.task_manager_service.empty()) {
      required[pending.pair.task_manager_service].insert(pending.pair.task_controller_name);
    }
  } else {
    for (const auto & arm : {pending.arm_a, pending.arm_b}) {
      if (arm.controller_action.empty()) {error = "No controller action for arm '" + arm.name + "'"; return false;}
      if (!arm.manager_service.empty()) {required[arm.manager_service].insert(arm.controller_name);}
    }
  }
  for (const auto & entry : required) {
    auto & client = managers_[entry.first];
    if (!client) {client = create_client<controller_manager_msgs::srv::ListControllers>(
        entry.first, rmw_qos_profile_services_default, clients_group_);}
    if (!client->wait_for_service(std::chrono::duration<double>(execution_options_.controller_wait_timeout))) {
      error = "Controller manager unavailable: " + entry.first; return false;
    }
    auto future = client->async_send_request(std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
    if (future.wait_for(std::chrono::duration<double>(execution_options_.acceptance_timeout)) != std::future_status::ready) {
      client->remove_pending_request(future); error = "Timed out checking controller readiness"; return false;
    }
    const auto response = future.get();
    for (const auto & name : entry.second) {
      const auto it = std::find_if(response->controller.begin(), response->controller.end(),
        [&name](const auto & controller) {return controller.name == name && controller.state == "active";});
      if (name.empty() || it == response->controller.end()) {
        error = "Operator must activate controller '" + name + "'"; return false;
      }
    }
  }
  return true;
}

bool PlanBimanualCartesianTrajectoryActionServer::validatePending(const PendingPlan & pending, std::string & error) const
{
  const auto canceled = [this, &pending]() {return stopped_.load() || pending.handle->is_canceling() || !pending.handle->is_active();};
  planning_scene::PlanningScenePtr scene;
  if (!context_->snapshot(pending.plan.joint_trajectory.joint_names, state_timeout_, state_max_age_, scene, error, canceled)) {return false;}
  const auto & measured = scene->getCurrentState();
  const auto & planned_start = pending.plan.trajectory->getFirstWayPoint();
  for (const auto & arm : {pending.arm_a, pending.arm_b}) {
    const auto * group = context_->model()->getJointModelGroup(arm.group_name);
    const auto * mount = group->getJointModels().front()->getParentLinkModel();
    if (!mount) {continue;}
    const auto & planned_mount = planned_start.getGlobalLinkTransform(mount);
    const auto & measured_mount = measured.getGlobalLinkTransform(mount);
    if ((measured_mount.translation() - planned_mount.translation()).norm() > base_position_tolerance_ ||
      Eigen::AngleAxisd(planned_mount.rotation().transpose() * measured_mount.rotation()).angle() > base_orientation_tolerance_)
    {
      error = "Plan obsolete: arm mount moved ('" + arm.name + "'); send a new goal";
      return false;
    }
  }
  const auto & first = pending.plan.joint_trajectory.points.front();
  for (std::size_t i = 0; i < pending.plan.joint_trajectory.joint_names.size(); ++i) {
    const auto & name = pending.plan.joint_trajectory.joint_names[i];
    const auto * joint = context_->model()->getJointOfVariable(name);
    const double tolerance = joint->getType() == moveit::core::JointModel::PRISMATIC ?
      start_prismatic_tolerance_ : start_joint_tolerance_;
    double delta = measured.getVariablePosition(name) - first.positions[i];
    if (joint->getType() == moveit::core::JointModel::REVOLUTE &&
      static_cast<const moveit::core::RevoluteJointModel *>(joint)->isContinuous()) {delta = std::remainder(delta, 2.0 * std::acos(-1.0));}
    if (std::abs(delta) > tolerance) {error = "Plan obsolete: initial joint moved ('" + name + "'); send a new goal"; return false;}
  }
  Eigen::Isometry3d current_transform;
  if (!visualization_->modelTransform(context_->model()->getModelFrame(), current_transform, error)) {return false;}
  if ((current_transform.translation() - pending.fixed_from_model.translation()).norm() > base_position_tolerance_ ||
    Eigen::AngleAxisd(pending.fixed_from_model.rotation().transpose() * current_transform.rotation()).angle() > base_orientation_tolerance_)
  {error = "Plan obsolete: robot base moved; send a new goal"; return false;}
  return bimanual_cartesian::BimanualCartesianPlanner().validatePlan(
    scene, pending.arm_a, pending.arm_b, pending.pair, pending.goal,
    pending.options, pending.plan, error, canceled);
}

void PlanBimanualCartesianTrajectoryActionServer::executePlan(const std::shared_ptr<PendingPlan> & pending)
{
  try {
    std::string error;
    if (!backendReady(*pending, error)) {
      if (stopped_.load() || pending->handle->is_canceling()) {finish(pending, false, "Execution request canceled");}
      else {retain(pending, error + "; plan retained. Activate the backend and call execute_pending_trajectory");}
      return;
    }
    if (!validatePending(*pending, error)) {finish(pending, false, error); return;}
    if (pending->backend == "pose") {executePosePlan(pending); return;}
    std::vector<ExecutionTarget> targets;
    auto options = execution_options_;
    if (pending->backend == "task") {
      targets.push_back({"bimanual_joint_task", pending->pair.task_action, {}});
      options.use_common_start_stamp = false;
      options.start_delay = 0.0;
    } else {
      for (const auto & arm : {pending->arm_a, pending->arm_b}) {
        targets.push_back({arm.name, arm.controller_action, arm.joint_names});
      }
    }
    feedback(pending->handle, 0.8F, "executing", "Sending joint trajectory to manually activated backend");
    const auto result = executor_->execute(pending->plan.joint_trajectory, targets, options,
      [this, pending]() {return stopped_.load() || !execution_active_.load() || pending->handle->is_canceling() || !pending->handle->is_active();},
      [this, pending]() {feedback(pending->handle, 0.85F, "executing", "Tracking coordinated joint trajectory");});
    if (!result.success && result.retryable && !stopped_.load() && !pending->handle->is_canceling()) {
      retain(pending, result.message + "; plan retained. Check the enabled joint trajectory task and retry the execute service");
    } else {finish(pending, result.success, result.message);}
  } catch (const std::exception & ex) {executor_->cancel(); finish(pending, false, std::string("Execution exception: ") + ex.what());}
}

bool PlanBimanualCartesianTrajectoryActionServer::setAutoUpdate(
  PoseEndpoint & endpoint, bool enabled, std::string & error)
{
  const double timeout = execution_options_.controller_wait_timeout;
  if (!endpoint.setter) {
    endpoint.setter = moveit_node_->create_client<rcl_interfaces::srv::SetParametersAtomically>(
      endpoint.controller_node + "/set_parameters_atomically");
  }
  if (!endpoint.setter->wait_for_service(std::chrono::duration<double>(timeout))) {
    error = "Parameter service unavailable: " + endpoint.controller_node; return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  do {
    auto input = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    input->parameters.push_back(rclcpp::Parameter(
        "tasks." + endpoint.task_id + ".auto_update_target_pose", enabled).to_parameter_msg());
    const double remaining = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
    auto future = endpoint.setter->async_send_request(input);
    if (future.wait_for(std::chrono::duration<double>(std::max(0.0, remaining))) != std::future_status::ready) {
      endpoint.setter->remove_pending_request(future);
      error = "Timed out changing auto_update_target_pose on " + endpoint.task_id; return false;
    }
    const auto response = future.get();
    if (response->result.successful) {return true;}
    error = response->result.reason;
    // The task priority controller rejects parameter updates while it is busy; retry until the deadline.
    const bool busy = error.find("retry the parameter change") != std::string::npos ||
      error.find("retry the parameter update") != std::string::npos;
    if (!busy) {return false;}
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}

bool PlanBimanualCartesianTrajectoryActionServer::checkPoseSamples(
  const std::shared_ptr<PendingPlan> & pending, PoseEndpoint & endpoint,
  std::vector<sura_manipulation_actions::msg::CartesianPoseSample> samples,
  bool & retryable, std::string & error)
{
  retryable = true;
  const auto canceled = [this, pending]() {
      return stopped_.load() || !execution_active_.load() ||
        pending->handle->is_canceling() || !pending->handle->is_active();
    };
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(stpik_timeout_);
  while (!endpoint.checker->action_server_is_ready()) {
    if (canceled()) {error = "STPIK canceled"; return false;}
    if (std::chrono::steady_clock::now() >= deadline) {
      error = "STPIK checker unavailable: " + endpoint.check_action; return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  CheckAction::Goal goal;
  // The checker compares this frame with the controller world frame.
  goal.trajectory.header.frame_id = context_->model()->getModelFrame();
  goal.samples = std::move(samples);
  goal.task_id = endpoint.task_id;
  goal.tip_frame = arms_.at(endpoint.arm_name).tip_frame;
  // Poses are absolute tip poses; the orientation tolerance is taken in the model frame.
  goal.tolerance_orientation.w = 1.0;
  std::copy_n(tip_path_tolerance_.begin(), 6, goal.path_tolerance.begin());
  std::copy_n(tip_goal_tolerance_.begin(), 6, goal.goal_tolerance.begin());
  goal.goal_time_tolerance = goal_time_tolerance_;
  goal.validation_id = 0;
  try {
    auto accepted = endpoint.checker->async_send_goal(goal);
    while (accepted.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
      if (canceled() || std::chrono::steady_clock::now() >= deadline) {
        error = "STPIK request canceled or timed out; feasibility unknown"; return false;
      }
    }
    const auto handle = accepted.get();
    if (!handle) {error = "STPIK request rejected: checker may be busy"; return false;}
    auto result = endpoint.checker->async_get_result(handle);
    while (result.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
      if (canceled() || std::chrono::steady_clock::now() >= deadline) {
        endpoint.checker->async_cancel_goal(handle);
        error = "STPIK simulation canceled or timed out; feasibility unknown"; return false;
      }
    }
    const auto wrapped = result.get();
    if (!wrapped.result) {error = "STPIK returned no diagnostic"; return false;}
    const auto & report = *wrapped.result;
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
      report.outcome == CheckAction::Result::SUCCESS)
    {return true;}
    retryable = report.outcome != CheckAction::Result::FAIL;
    std::ostringstream detail;
    detail << "STPIK " << report.code << " at t=" << report.time_from_start << " s: " << report.message;
    if (!report.task_id.empty()) {detail << "; task=" << report.task_id;}
    if (!report.constraint.empty()) {detail << "; constraint=" << report.constraint;}
    error = detail.str();
    return false;
  } catch (const std::exception & exception) {
    error = "STPIK communication failed: " + std::string(exception.what());
    return false;
  }
}

void PlanBimanualCartesianTrajectoryActionServer::executePosePlan(const std::shared_ptr<PendingPlan> & pending)
{
  std::string error;
  std::vector<PoseEndpoint *> disabled;
  const auto canceled = [this, pending]() {
      return stopped_.load() || !execution_active_.load() ||
        pending->handle->is_canceling() || !pending->handle->is_active();
    };
  const auto restore = [this, &disabled]() {
      for (auto * endpoint : disabled) {
        std::string restore_error;
        if (!setAutoUpdate(*endpoint, true, restore_error)) {
          RCLCPP_ERROR(get_logger(), "Cannot restore auto_update_target_pose on %s: %s",
            endpoint->task_id.c_str(), restore_error.c_str());
        }
      }
      disabled.clear();
    };
  try {
    const auto & trajectory = pending->plan.joint_trajectory;
    if (trajectory.points.size() < 2) {throw std::runtime_error("Pose backend needs at least two trajectory points");}
    const auto & names = trajectory.joint_names;
    const auto & robot = context_->model();
    // Pose tasks track absolute tip poses in the model (world) frame.
    const std::string reference_frame = robot->getModelFrame();
    const auto seconds = [](const builtin_interfaces::msg::Duration & duration) {
        return rclcpp::Duration(duration).seconds();
      };
    const double duration = seconds(trajectory.points.back().time_from_start);

    const auto sample_pose = [&](double elapsed, const std::string & arm_name) {
        // Linear interpolation of the planned joints, then forward kinematics.
        std::size_t index = 1;
        while (index + 1 < trajectory.points.size() && seconds(trajectory.points[index].time_from_start) < elapsed) {++index;}
        const auto & a = trajectory.points[index - 1];
        const auto & b = trajectory.points[index];
        const double ta = seconds(a.time_from_start);
        const double tb = seconds(b.time_from_start);
        const double s = tb > ta ? std::clamp((elapsed - ta) / (tb - ta), 0.0, 1.0) : 1.0;
        std::vector<double> positions(a.positions.size());
        for (std::size_t j = 0; j < positions.size(); ++j) {
          positions[j] = a.positions[j] + s * (b.positions[j] - a.positions[j]);
        }
        moveit::core::RobotState state(robot);
        state.setVariablePositions(names, positions);
        state.update();
        return state.getGlobalLinkTransform(arms_.at(arm_name).tip_frame);
      };
    // Samples for STPIK: the same poses that are streamed, on a 10 ms grid ending at the goal time.
    const double check_step = 0.01;
    std::vector<double> times;
    for (double t = 0.0; t < duration; t += check_step) {times.push_back(t);}
    times.push_back(duration);
    const auto samples_for = [&](const std::string & arm_name) {
        std::vector<sura_manipulation_actions::msg::CartesianPoseSample> samples;
        samples.reserve(times.size());
        for (const double t : times) {
          const auto pose = sample_pose(t, arm_name);
          sura_manipulation_actions::msg::CartesianPoseSample sample;
          sample.time_from_start = t;
          sample.pose.position.x = pose.translation().x();
          sample.pose.position.y = pose.translation().y();
          sample.pose.position.z = pose.translation().z();
          const Eigen::Quaterniond q(pose.rotation());
          sample.pose.orientation.x = q.x(); sample.pose.orientation.y = q.y();
          sample.pose.orientation.z = q.z(); sample.pose.orientation.w = q.w();
          samples.push_back(sample);
        }
        return samples;
      };
    if (duration <= 0.0) {throw std::runtime_error("Pose backend needs a positive trajectory duration");}
    for (const auto & arm : {pending->arm_a, pending->arm_b}) {
      if (!pending->use_stpik) {break;}
      bool retryable = false;
      feedback(pending->handle, 0.8F, "stpik_check", "Validating " + arm.name + " pose trajectory with STPIK");
      if (!checkPoseSamples(pending, pose_endpoints_.at(arm.name), samples_for(arm.name), retryable, error)) {
        if (canceled()) {finish(pending, false, "Pose execution canceled during STPIK");}
        else if (retryable) {retain(pending, error + "; plan retained; retry execution");}
        else {finish(pending, false, error);}
        return;
      }
    }

    feedback(pending->handle, 0.8F, "pose_setup", "Disabling automatic pose capture on pose tasks");
    for (const auto & arm : {pending->arm_a, pending->arm_b}) {
      auto & endpoint = pose_endpoints_.at(arm.name);
      if (canceled()) {throw std::runtime_error("Pose execution canceled");}
      if (!setAutoUpdate(endpoint, false, error)) {throw std::runtime_error("Cannot disable automatic pose capture: " + error);}
      disabled.push_back(&endpoint);
    }
    if (!validatePending(*pending, error)) {throw std::runtime_error(error);}

    // Central differences on the sampled poses give the feedforward twist in reference axes.
    const double h = 1e-3;
    const auto publish = [&](double elapsed, rclcpp::Time stamp) {
        const double before = std::max(0.0, elapsed - h);
        const double after = std::min(duration, elapsed + h);
        for (const auto & arm : {pending->arm_a, pending->arm_b}) {
          const auto pose = sample_pose(elapsed, arm.name);
          geometry_msgs::msg::PoseStamped message;
          message.header.frame_id = reference_frame;
          message.header.stamp = stamp;
          message.pose.position.x = pose.translation().x();
          message.pose.position.y = pose.translation().y();
          message.pose.position.z = pose.translation().z();
          const Eigen::Quaterniond q(pose.rotation());
          message.pose.orientation.x = q.x(); message.pose.orientation.y = q.y();
          message.pose.orientation.z = q.z(); message.pose.orientation.w = q.w();
          auto & endpoint = pose_endpoints_.at(arm.name);
          endpoint.target->publish(message);

          const auto ahead = sample_pose(after, arm.name);
          const auto behind = sample_pose(before, arm.name);
          const double span = after - before;
          geometry_msgs::msg::TwistStamped twist;
          twist.header.frame_id = reference_frame;
          twist.header.stamp = stamp;
          if (span > 0.0) {
            const Eigen::Vector3d linear = (ahead.translation() - behind.translation()) / span;
            const Eigen::AngleAxisd rotation(ahead.rotation() * behind.rotation().transpose());
            const Eigen::Vector3d angular = rotation.axis() * rotation.angle() / span;
            twist.twist.linear.x = linear.x(); twist.twist.linear.y = linear.y(); twist.twist.linear.z = linear.z();
            twist.twist.angular.x = angular.x(); twist.twist.angular.y = angular.y(); twist.twist.angular.z = angular.z();
          }
          endpoint.feedforward->publish(twist);
        }
      };
    const auto publish_zero_feedforward = [this, &pending]() {
        for (const auto & arm : {pending->arm_a, pending->arm_b}) {
          geometry_msgs::msg::TwistStamped zero;
          zero.header.frame_id = context_->model()->getModelFrame();
          zero.header.stamp = now();
          const auto it = pose_endpoints_.find(arm.name);
          if (it != pose_endpoints_.end() && it->second.feedforward) {it->second.feedforward->publish(zero);}
        }
      };

    const auto start = now();
    const auto period = std::chrono::duration<double>(1.0 / execution_options_.feedback_rate);
    while (!canceled()) {
      const double elapsed = (now() - start).seconds();
      publish(std::min(elapsed, duration), now());
      auto output = std::make_shared<Action::Feedback>();
      output->progress = static_cast<float>(0.8 + 0.2 * (duration > 0.0 ? std::clamp(elapsed / duration, 0.0, 1.0) : 1.0));
      output->current_stage = elapsed >= duration ? "converging_pose" : "tracking_pose";
      output->message = "Streaming planned poses to left_pose and right_pose";
      pending->handle->publish_feedback(output);
      if (elapsed >= duration + execution_options_.execution_wait_margin) {break;}
      std::this_thread::sleep_for(period);
    }
    publish_zero_feedforward();
    restore();
    if (canceled()) {finish(pending, false, "Pose execution canceled");}
    else {finish(pending, true, "Planned poses streamed to left_pose and right_pose; auto-update restored");}
  } catch (const std::exception & ex) {
    restore();
    finish(pending, false, std::string("Pose execution exception: ") + ex.what());
  }
}

void PlanBimanualCartesianTrajectoryActionServer::executeTrigger(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!execution_active_.load()) {
    response->message = "Activate the Cartesian planner lifecycle before requesting execution";
    return;
  }
  std::shared_ptr<PendingPlan> pending;
  {std::lock_guard<std::mutex> lock(mutex_);
    if (stage_ != Stage::Pending || !pending_) {response->message = "No pending plan is available"; return;}
    pending = pending_; stage_ = Stage::Executing;}
  if (worker_.joinable()) {worker_.join();}
  if (pending->handle->is_canceling() || !pending->handle->is_active()) {
    finish(pending, false, "Execution request canceled");
    response->message = "Execution request canceled";
    return;
  }
  stopped_.store(false);
  response->success = true;
  response->message = "Requested pending trajectory execution; readiness and current scene will be checked";
  worker_ = std::thread([this, pending]() {executePlan(pending);});
}

void PlanBimanualCartesianTrajectoryActionServer::cancelTrigger(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::shared_ptr<PendingPlan> pending;
  Stage stage;
  {std::lock_guard<std::mutex> lock(mutex_); pending = pending_; stage = stage_;}
  if (!pending) {response->message = "No pending or active Cartesian goal"; return;}
  stopped_.store(true); executor_->cancel();
  if (stage == Stage::Pending) {finish(pending, false, "Plan canceled by operator service");}
  response->success = true; response->message = "Cancellation requested";
}

void PlanBimanualCartesianTrajectoryActionServer::feedback(
  const std::shared_ptr<Handle> & handle, float progress,
  const std::string & stage, const std::string & message) const
{
  if (!handle || !handle->is_active()) {return;}
  auto feedback = std::make_shared<Action::Feedback>();
  feedback->progress = progress; feedback->current_stage = stage; feedback->message = message;
  try {
    handle->publish_feedback(feedback);
  } catch (const std::exception & ex) {
    RCLCPP_DEBUG(get_logger(), "Cartesian feedback unavailable: %s", ex.what());
  }
}

void PlanBimanualCartesianTrajectoryActionServer::retain(
  const std::shared_ptr<PendingPlan> & pending, const std::string & message)
{
  {std::lock_guard<std::mutex> lock(mutex_); if (pending_ != pending) {return;} stage_ = Stage::Pending;}
  feedback(pending->handle, 0.8F, "waiting_for_execute", message);
}

void PlanBimanualCartesianTrajectoryActionServer::finish(
  const std::shared_ptr<PendingPlan> & pending, bool success, const std::string & message)
{
  if (pending->finished.exchange(true)) {return;}
  auto result = std::make_shared<Action::Result>();
  result->success = success;
  result->message = message;
  result->joint_trajectory = pending->plan.joint_trajectory;
  result->arm_a_path = pending->views.arm_a_path;
  result->arm_b_path = pending->views.arm_b_path;
  result->relative_path = pending->views.relative_path;
  result->desired_relative_path = pending->views.desired_relative_path;
  {std::lock_guard<std::mutex> lock(mutex_);
    if (pending_ == pending || !pending_) {pending_.reset(); stage_ = Stage::Idle;}}
  try {
    if (pending->handle && pending->handle->is_active()) {
      if (pending->handle->is_canceling()) {result->success = false; pending->handle->canceled(result);}
      else if (success) {feedback(pending->handle, 1.0F, "completed", message); pending->handle->succeed(result);}
      else {pending->handle->abort(result);}
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN(get_logger(), "Could not report Cartesian result: %s", ex.what());
  }
}

void PlanBimanualCartesianTrajectoryActionServer::stop(const std::string & message)
{
  stopped_.store(true);
  if (executor_) {executor_->cancel();}
  if (worker_.joinable()) {worker_.join();}
  std::shared_ptr<PendingPlan> pending;
  {std::lock_guard<std::mutex> lock(mutex_); pending = pending_;}
  if (pending) {finish(pending, false, message);}
}

void PlanBimanualCartesianTrajectoryActionServer::cleanupResources()
{
  execution_active_.store(false);
  stop("Cartesian planner cleaned up");
  cancel_timer_.reset(); action_.reset(); execute_service_.reset(); cancel_service_.reset();
  if (context_) {context_->stop();}
  managers_.clear(); clients_group_.reset(); executor_.reset(); visualization_.reset(); context_.reset(); moveit_node_.reset();
  arms_.clear(); pairs_.clear(); pose_endpoints_.clear();
  planner_options_.tightened_limits.clear(); planner_options_.nominal_positions.clear();
}

}  // namespace sura_manipulation_actions

#ifndef SURA_MANIPULATION_ACTIONS_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sura_manipulation_actions::PlanBimanualCartesianTrajectoryActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
#endif
