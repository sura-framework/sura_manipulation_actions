#include "sura_manipulation_actions/plan_cartesian_trajectory_action_server.hpp"
#include "sura_manipulation_actions/cartesian_tracking.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>
#include <thread>
#include <sstream>

#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"

namespace sura_manipulation_actions
{
namespace
{
using GetParameters = rcl_interfaces::srv::GetParameters;
using SetParameters = rcl_interfaces::srv::SetParametersAtomically;
using PoseError = Eigen::Matrix<double, 6, 1>;

template<class ActionT, class Deadline>
bool controllerRequest(const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
  const typename ActionT::Goal & goal, const Deadline & deadline, const std::function<bool()> & canceled,
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & wrapped, std::string & error)
{
  // Handle late acceptance too, including cancellation while send_goal is in flight.
  struct Dispatch {std::mutex mutex; bool canceled = false; typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr handle;};
  auto dispatch = std::make_shared<Dispatch>();
  typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
  options.goal_response_callback = [dispatch, client](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr handle) {
      bool cancel = false;
      {std::scoped_lock lock(dispatch->mutex); dispatch->handle = handle; cancel = dispatch->canceled;}
      if (cancel && handle) {try {client->async_cancel_goal(handle);} catch (const std::exception &) {}}
    };
  const auto cancel_check = [&]() {
      typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr handle;
      {std::scoped_lock lock(dispatch->mutex); dispatch->canceled = true; handle = dispatch->handle;}
      if (handle) {try {client->async_cancel_goal(handle);} catch (const std::exception &) {}}
    };
  try {
    auto accepted = client->async_send_goal(goal, options);
    while (accepted.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
      if (canceled() || std::chrono::steady_clock::now() >= deadline) {
        cancel_check(); error = "Controller action request canceled or timed out; feasibility unknown"; return false;
      }
    }
    const auto handle = accepted.get();
    if (!handle) {error = "Controller action request rejected: controller may be busy"; return false;}
    auto result = client->async_get_result(handle);
    while (result.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
      if (canceled() || std::chrono::steady_clock::now() >= deadline) {
        cancel_check(); error = "Controller action canceled or timed out; feasibility unknown"; return false;
      }
    }
    wrapped = result.get();
    return true;
  } catch (const std::exception & e) {
    cancel_check(); error = e.what(); return false;
  }
}

template<class Goal, class Plan>
void setTrackingGoal(Goal & goal, const Plan & pending, const std::string & frame, const std::string & task_id)
{
  goal.trajectory = pending.pose_path.descriptor(frame);
  goal.task_id = task_id;
  goal.tip_frame = pending.arm.tip_frame;
  const Eigen::Quaterniond tolerance(pending.goal.world_from_tolerance.rotation());
  goal.tolerance_orientation.x = tolerance.x(); goal.tolerance_orientation.y = tolerance.y();
  goal.tolerance_orientation.z = tolerance.z(); goal.tolerance_orientation.w = tolerance.w();
  std::copy_n(pending.tracking_tolerances.tip_path.begin(), 6, goal.path_tolerance.begin());
  std::copy_n(pending.tracking_tolerances.tip_goal.begin(), 6, goal.goal_tolerance.begin());
  goal.goal_time_tolerance = rclcpp::Duration(pending.tracking_tolerances.goal_time_tolerance).seconds();
}

template<class Service>
typename Service::Response::SharedPtr request(
  const typename rclcpp::Client<Service>::SharedPtr & client,
  const typename Service::Request::SharedPtr & input, double timeout,
  std::string & error, const std::function<bool()> & canceled = {})
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  while (!client->service_is_ready()) {
    if (!rclcpp::ok() || (canceled && canceled())) {error = "Pose task preparation canceled"; return {};}
    if (std::chrono::steady_clock::now() >= deadline) {
      error = "Pose task parameter service unavailable: " + std::string(client->get_service_name()); return {};
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  auto future = client->async_send_request(input);
  while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    if (!rclcpp::ok() || (canceled && canceled()) || std::chrono::steady_clock::now() >= deadline) {
      client->remove_pending_request(future);
      error = "Pose task parameter request interrupted or timed out: " + std::string(client->get_service_name());
      return {};
    }
  }
  return future.get();
}

bool setAutoUpdate(
  const rclcpp::Client<SetParameters>::SharedPtr & client,
  const std::string & name, bool enabled, double timeout, std::string & error)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  do {
    auto input = std::make_shared<SetParameters::Request>();
    input->parameters.push_back(rclcpp::Parameter(name, enabled).to_parameter_msg());
    const double remaining = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
    const auto response = request<SetParameters>(client, input, std::max(0.0, remaining), error);
    if (!response) {return false;}
    if (response->result.successful) {return true;}
    error = response->result.reason;
    // Both hosts can briefly reject parameters while updating tasks.
    const bool busy = error.find("retry the parameter change") != std::string::npos ||
      error.find("retry the parameter update") != std::string::npos;
    if (!busy) {return false;}
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}

Eigen::Isometry3d transformPose(const geometry_msgs::msg::Transform & value)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() << value.translation.x, value.translation.y, value.translation.z;
  Eigen::Quaterniond rotation(value.rotation.w, value.rotation.x, value.rotation.y, value.rotation.z);
  if (!result.translation().allFinite() || !rotation.coeffs().allFinite() || rotation.norm() < 1e-12) {
    throw std::runtime_error("Non-finite pose or invalid quaternion in TF");
  }
  result.linear() = rotation.normalized().toRotationMatrix();
  return result;
}

geometry_msgs::msg::PoseStamped poseMessage(
  const Eigen::Isometry3d & value, const std::string & frame, const rclcpp::Time & time)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = frame; result.header.stamp = time;
  result.pose.position.x = value.translation().x();
  result.pose.position.y = value.translation().y();
  result.pose.position.z = value.translation().z();
  const Eigen::Quaterniond q(value.rotation());
  result.pose.orientation.x = q.x(); result.pose.orientation.y = q.y();
  result.pose.orientation.z = q.z(); result.pose.orientation.w = q.w();
  return result;
}

PoseError poseError(
  const Eigen::Isometry3d & desired, const Eigen::Isometry3d & measured,
  const Eigen::Matrix3d & reference, const PoseError & activation = PoseError::Ones())
{
  return cartesianTrackingError(desired, measured, reference, activation);
}

bool within(const PoseError & error, const std::vector<double> & tolerance)
{
  for (Eigen::Index i = 0; i < error.size(); ++i) {
    if (!std::isfinite(error(i)) || std::abs(error(i)) > tolerance[static_cast<std::size_t>(i)] + 1e-9) {return false;}
  }
  return true;
}
}  // namespace

bool PlanCartesianTrajectoryActionServer::readTipPose(
  const PendingPlan & pending, Eigen::Isometry3d & pose, std::string & error) const
{
  try {
    const auto transform = pose_tf_buffer_->lookupTransform(reference_frame_, pending.arm.tip_frame, tf2::TimePointZero);
    const auto & stamp = transform.header.stamp;
    if (stamp.sec != 0 || stamp.nanosec != 0) {
      const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
      if (std::abs(age) > state_max_age_) {error = "Tool TF is stale or ahead of ROS time"; return false;}
    }
    pose = transformPose(transform.transform);
    return true;
  } catch (const std::exception & exception) {
    error = "Tool TF unavailable: " + std::string(exception.what()); return false;
  }
}

void PlanCartesianTrajectoryActionServer::planPoseGoal(const std::shared_ptr<PendingPlan> & pending)
{
  try {
    std::string error;
    const auto canceled = [this, &pending]() {
        return stopped_.load() || pending->handle->is_canceling() || !pending->handle->is_active();
      };
    feedback(pending->handle, 0.0F, "acquiring_pose", "Reading tool TF; generating a geometric pose reference");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(state_timeout_);
    while (!readTipPose(*pending, pending->pose_start, error)) {
      if (canceled() || std::chrono::steady_clock::now() >= deadline) {finish(pending, false, error); return;}
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto input = pending->handle->get_goal()->target_pose;
    if (input.header.frame_id.empty()) {input.header.frame_id = reference_frame_;}
    Eigen::Isometry3d world_from_input = Eigen::Isometry3d::Identity();
    if (input.header.frame_id != reference_frame_) {
      world_from_input = transformPose(pose_tf_buffer_->lookupTransform(
        reference_frame_, input.header.frame_id,
        rclcpp::Time(input.header.stamp, get_clock()->get_clock_type())).transform);
    }
    pending->goal.relative_target = world_from_input * pending->goal.relative_target;
    pending->goal.world_from_tolerance = world_from_input;
    const auto & options = pending->options;
    if (!pending->pose_path.init(pending->pose_start, pending->goal.relative_target,
        pose_linear_velocity_ * options.velocity_scaling,
        pose_angular_velocity_ * options.velocity_scaling,
        pose_linear_acceleration_ * options.acceleration_scaling,
        pose_angular_acceleration_ * options.acceleration_scaling,
        pending->goal.cartesian_step, options.angular_step,
        static_cast<std::size_t>(std::max(0, options.max_waypoints)), error))
    {finish(pending, false, error); return;}
    if (canceled()) {finish(pending, false, "Pose planning canceled"); return;}
    auto desired = pending->pose_path.desiredPath(reference_frame_);
    desired.header.stamp = now();
    for (auto & point : desired.poses) {point.header.stamp = desired.header.stamp;}
    pending->views = visualization_->publishPosePath(desired);
    if ((pending->handle->get_goal()->optimize_initial_posture || pending->wants_nominal) &&
      !preparePosture(*pending, false, error))
    {
      finish(pending, false, error); return;
    }
    retain(pending, pending->handle->get_goal()->use_stpik ?
      "Geometric tool path published; STPIK validation will run when execution is requested" :
      "Geometric tool path published; use_stpik=false; waiting for Pose execution");
    if (canceled()) {finish(pending, false, "Pose planning canceled"); return;}
    if (pending->handle->get_goal()->execute && !canceled()) {
      {std::lock_guard<std::mutex> lock(mutex_);
        if (pending_ != pending || stage_ != Stage::Pending) {return;}
        stage_ = Stage::Executing;}
      executePosePlan(pending);
    }
  } catch (const std::exception & exception) {
    finish(pending, false, "Pose planning exception: " + std::string(exception.what()));
  }
}

bool PlanCartesianTrajectoryActionServer::validatePosePending(const PendingPlan & pending, std::string & error) const
{
  Eigen::Isometry3d measured;
  if (!readTipPose(pending, measured, error)) {return false;}
  if (!within(poseError(pending.pose_start, measured, pending.goal.world_from_tolerance.rotation(),
      pending.pose_activation),
      pending.tracking_tolerances.tip_path))
  {error = "Plan obsolete: tool moved since planning; send a new goal"; return false;}
  return true;
}

bool PlanCartesianTrajectoryActionServer::poseBackendReady(
  PendingPlan & pending, bool & auto_update, std::string & error)
{
  if (!backendReady(pending, error)) {return false;}
  const auto & endpoint = pose_endpoints_.at(pending.arm.name);
  const auto client = moveit_node_->create_client<GetParameters>(endpoint.controller_node + "/get_parameters");
  auto input = std::make_shared<GetParameters::Request>();
  const auto prefix = "tasks." + endpoint.task_id + ".";
  input->names = {"world_frame", prefix + "frame_id", prefix + "plugin", prefix + "auto_update_target_pose"};
  const auto response = request<GetParameters>(client, input, execution_options_.controller_wait_timeout, error,
    [this, &pending]() {return stopped_.load() || pending.handle->is_canceling() || !execution_active_.load();});
  if (!response) {return false;}
  const auto & values = response->values;
  using Type = rcl_interfaces::msg::ParameterType;
  if (values.size() != 4 || values[0].type != Type::PARAMETER_STRING ||
    values[1].type != Type::PARAMETER_STRING || values[2].type != Type::PARAMETER_STRING ||
    values[3].type != Type::PARAMETER_BOOL)
  {error = "Pose task parameters are missing or have invalid types"; return false;}
  if (values[0].string_value != reference_frame_ || values[1].string_value != pending.arm.tip_frame) {
    error = "Pose task world/tool frames must match planner world/tool frames"; return false;
  }
  if (values[2].string_value != "task_priority_kinematic_control/FramePoseTask" &&
    values[2].string_value != "task_priority_kinematic_control/EndEffectorPoseTask")
  {error = "Selected task must be FramePoseTask or EndEffectorPoseTask"; return false;}
  auto_update = values[3].bool_value;
  pending.pose_activation.setOnes();
  if (values[2].string_value == "task_priority_kinematic_control/FramePoseTask") {
    auto mask_request = std::make_shared<GetParameters::Request>();
    mask_request->names = {prefix + "activation"};
    const auto mask_response = request<GetParameters>(client, mask_request,
      execution_options_.controller_wait_timeout, error,
      [this, &pending]() {return stopped_.load() || pending.handle->is_canceling() || !execution_active_.load();});
    if (!mask_response) {return false;}
    if (mask_response->values.size() != 1 ||
      mask_response->values[0].type != Type::PARAMETER_INTEGER_ARRAY)
    {error = "Frame pose activation must be an integer array"; return false;}
    const auto & mask = mask_response->values[0].integer_array_value;
    if (!mask.empty() && mask.size() != 6)
    {error = "Frame pose activation must contain six values or be empty"; return false;}
    for (size_t i = 0; i < mask.size(); ++i) {
      if (mask[i] != 0 && mask[i] != 1)
      {error = "Frame pose activation values must be 0 or 1"; return false;}
      pending.pose_activation(i) = mask[i];
    }
    if (pending.pose_activation.isZero())
    {error = "Frame pose task has no active axes"; return false;}
  }
  if (endpoint.target->get_subscription_count() == 0 || endpoint.feedforward->get_subscription_count() == 0) {
    error = "Pose target/feedforward subscribers are unavailable; check controller namespace and task ID"; return false;
  }
  if (pending.wants_nominal && (!endpoint.nominal || endpoint.nominal->get_subscription_count() == 0)) {
    error = "robot_nominal reference subscriber is unavailable on " + endpoint.nominal_topic +
      "; configure a RobotNominalTask in the controller"; return false;
  }
  return true;
}

bool PlanCartesianTrajectoryActionServer::preparePosture(PendingPlan & pending, bool apply, std::string & error)
{
  const auto & endpoint = pose_endpoints_.at(pending.arm.name);
  const auto canceled = [&]() {
    return !rclcpp::ok() || stopped_.load() || pending.handle->is_canceling() ||
           !pending.handle->is_active() || (apply && !execution_active_.load());
  };
  if (!endpoint.preparer->wait_for_action_server(std::chrono::seconds(2)) || canceled()) {
    error = "Posture action unavailable: " + endpoint.prepare_action; return false;
  }
  feedback(pending.handle, apply ? 0.8F : 0.4F, apply ? "applying_posture" : "optimizing_posture",
    apply ? "Rechecking and applying the proposed configuration" :
    pending.wants_nominal ? "Optimizing the initial posture and the whole-body nominal trajectory" :
    "Evaluating initial postures over the Cartesian path");
  PrepareAction::Goal goal;
  setTrackingGoal(goal, pending, reference_frame_, endpoint.task_id);
  goal.operation = apply ? goal.APPLY : goal.PLAN;
  goal.preparation_id = apply ? pending.proposal.preparation_id : 0;
  goal.full_trajectory = pending.wants_nominal;
  planning_scene::PlanningScenePtr scene;
  if (!context_->snapshot(pending.arm.joint_names, state_timeout_, state_max_age_, scene, error, canceled)) {return false;}
  scene->getPlanningSceneMsg(goal.scene);
  if (!moveit_node_->get_parameter("robot_description", goal.robot_description) ||
      !moveit_node_->get_parameter("robot_description_semantic", goal.robot_description_semantic)) {
    error = "Posture optimization requires the MoveIt URDF and SRDF"; return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
  rclcpp_action::ClientGoalHandle<PrepareAction>::WrappedResult wrapped;
  if (!controllerRequest<PrepareAction>(endpoint.preparer, goal, deadline, canceled, wrapped, error)) {return false;}
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result || !wrapped.result->success ||
      !wrapped.result->proposal.preparation_id) {
    error = wrapped.result ? wrapped.result->message : "Missing posture result"; return false;
  }
  if (pending.wants_nominal) {
    WholeBodyNominal nominal;
    if (!nominal.init(wrapped.result->nominal_joints, wrapped.result->nominal_base, error)) {
      error = "Invalid nominal trajectory from the controller: " + error; return false;
    }
    if (nominal.frame() != reference_frame_) {
      error = "Nominal trajectory frame differs from the planning frame"; return false;
    }
    pending.nominal = std::move(nominal);
    pending.nominal_joints = wrapped.result->nominal_joints;
    pending.nominal_base = wrapped.result->nominal_base;
    pending.plan.joint_trajectory = pending.nominal_joints;
    pending.plan.base_trajectory = pending.nominal_base;
    visualization_->publishPoseNominal(pending.nominal_joints, pending.nominal_base, pending.views);
    feedback(pending.handle, apply ? 0.8F : 0.5F, apply ? "nominal_replanned" : "nominal_planned",
      "Whole-body nominal (" + wrapped.result->nominal_method + ", cost " +
      std::to_string(wrapped.result->nominal_cost) + ", " + std::to_string(pending.nominal_joints.points.size()) +
      " knots over " + std::to_string(pending.nominal.duration()) + " s)", &wrapped.result->proposal);
  }
  pending.proposal = wrapped.result->proposal;
  pending.posture_applied = apply;
  return true;
}

bool PlanCartesianTrajectoryActionServer::checkPoseTrajectory(
  const PendingPlan & pending, uint64_t & validation_id, bool & retryable, std::string & error)
{
  retryable = true;
  const auto client = pose_endpoints_.at(pending.arm.name).checker;
  const auto canceled = [this, &pending]() {
      return !rclcpp::ok() || stopped_.load() || !execution_active_.load() ||
        pending.handle->is_canceling() || !pending.handle->is_active();
    };
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(stpik_timeout_);
  while (!client->action_server_is_ready()) {
    if (canceled()) {error = "STPIK canceled"; return false;}
    if (std::chrono::steady_clock::now() >= deadline) {
      error = "STPIK checker unavailable: " + pose_endpoints_.at(pending.arm.name).check_action; return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  CheckAction::Goal goal;
  setTrackingGoal(goal, pending, reference_frame_, pose_endpoints_.at(pending.arm.name).task_id);
  goal.validation_id = validation_id;
  if (pending.wants_nominal) {
    goal.nominal_joints = pending.nominal_joints;
    goal.nominal_base = pending.nominal_base;
  }
  CheckHandle::WrappedResult wrapped;
  if (!controllerRequest<CheckAction>(client, goal, deadline, canceled, wrapped, error)) {return false;}
  if (!wrapped.result) {error = "STPIK returned no diagnostic"; return false;}
  const auto & report = *wrapped.result;
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
    report.outcome == CheckAction::Result::SUCCESS && report.validation_id != 0)
  {validation_id = report.validation_id; return true;}
  retryable = report.outcome != CheckAction::Result::FAIL;
  std::ostringstream detail;
  detail << "STPIK " << report.code << " at t=" << report.time_from_start << " s: " << report.message;
  if (!report.task_id.empty()) {detail << "; task=" << report.task_id;}
  if (!report.constraint.empty()) {detail << "; constraint=" << report.constraint;}
  detail << "; q=[";
  for (size_t i = 0; i < report.joint_positions.size(); ++i) {
    if (i) {detail << ", ";}
    if (i < report.joint_names.size()) {detail << report.joint_names[i] << '=';}
    detail << report.joint_positions[i];
  }
  detail << "]; tip_error=[";
  for (size_t i = 0; i < report.tracking_error.size(); ++i) {
    if (i) {detail << ", ";} detail << report.tracking_error[i];
  }
  detail << "]; min_clearance=" << report.minimum_clearance
         << "; max_velocity_ratio=" << report.maximum_velocity_ratio;
  error = detail.str();
  if (report.outcome == CheckAction::Result::FAIL && !canceled()) {
    auto output = std::make_shared<Action::Feedback>();
    output->progress = 0.8F;
    output->current_stage = "stpik_failed";
    output->message = error;
    std::copy(report.tracking_error.begin(), report.tracking_error.end(), output->tip_error.begin());
    try {pending.handle->publish_feedback(output);}
    catch (const std::exception & exception) {
      RCLCPP_DEBUG(get_logger(), "STPIK failure feedback unavailable: %s", exception.what());
    }
  }
  return false;
}

void PlanCartesianTrajectoryActionServer::executePosePlan(const std::shared_ptr<PendingPlan> & pending)
{
  std::string error;
  bool auto_update = true;
  bool touched = false;
  bool success = false;
  Eigen::Isometry3d measured = pending->pose_start;
  const auto & endpoint = pose_endpoints_.at(pending->arm.name);
  const auto parameter_name = "tasks." + endpoint.task_id + ".auto_update_target_pose";
  rclcpp::Client<SetParameters>::SharedPtr setter;
  const auto canceled = [this, &pending]() {
      return !rclcpp::ok() || stopped_.load() || !execution_active_.load() ||
        pending->handle->is_canceling() || !pending->handle->is_active();
    };
  double last_nominal_time = 0.0;
  const auto publish_zero = [&]() {
      geometry_msgs::msg::TwistStamped zero;
      zero.header.frame_id = reference_frame_; zero.header.stamp = now();
      endpoint.feedforward->publish(zero);
    };
  try {
    if (!poseBackendReady(*pending, auto_update, error)) {
      if (canceled()) {finish(pending, false, "Pose execution canceled");}
      else {retain(pending, error + "; plan retained. Check the pose task and retry execution");}
      return;
    }
    if (!validatePosePending(*pending, error)) {finish(pending, false, error); return;}
    if (pending->wants_nominal && pending->nominal.empty()) {
      finish(pending, false, "Whole-body nominal trajectory is unavailable"); return;
    }
    if ((pending->handle->get_goal()->optimize_initial_posture || pending->wants_nominal) &&
      !pending->posture_applied &&
        (!preparePosture(*pending, true, error) || !validatePosePending(*pending, error))) {
      finish(pending, false, error); return;
    }
    if (pending->handle->get_goal()->use_stpik) {
      // This gate precedes ALL target/feedforward publications and parameter writes.
      uint64_t validation_id = 0;
      bool retryable = false;
      feedback(pending->handle, 0.8F, "stpik_check", "Validating the trajectory with isolated Task Priority simulation");
      if (!checkPoseTrajectory(*pending, validation_id, retryable, error)) {
        if (canceled()) {finish(pending, false, "Pose execution canceled during STPIK");}
        else if (retryable) {retain(pending, error + "; plan retained; retry execution");}
        else {finish(pending, false, error);}
        return;
      }
      // The hardware and effective task settings can change while checking.
      if (!poseBackendReady(*pending, auto_update, error)) {
        if (canceled()) {finish(pending, false, "Pose execution canceled");}
        else {retain(pending, error + "; plan retained; retry execution");}
        return;
      }
      if (!validatePosePending(*pending, error)) {finish(pending, false, error); return;}
      if (!checkPoseTrajectory(*pending, validation_id, retryable, error)) {
        if (canceled()) {finish(pending, false, "Pose execution canceled during STPIK revalidation");}
        else if (retryable) {retain(pending, error + "; plan retained; retry execution");}
        else {finish(pending, false, error);}
        return;
      }
    }
    setter = moveit_node_->create_client<SetParameters>(endpoint.controller_node + "/set_parameters_atomically");
    if (canceled()) {throw std::runtime_error("Pose execution canceled");}
    if (!readTipPose(*pending, measured, error)) {throw std::runtime_error(error);}
    // A response can time out after a remote change; always attempt restoration.
    touched = true;
    publish_zero();
    endpoint.target->publish(poseMessage(measured, reference_frame_, now()));
    if (!setAutoUpdate(setter, parameter_name, false, execution_options_.controller_wait_timeout, error)) {
      throw std::runtime_error("Cannot disable automatic pose capture: " + error);
    }
    if (canceled()) {throw std::runtime_error("Pose execution canceled");}
    if (!validatePosePending(*pending, error)) {throw std::runtime_error(error);}
    const auto start = now();
    auto previous = start;
    auto next_tick = std::chrono::steady_clock::now();
    const double duration = pending->pose_path.duration();
    const double allowance = rclcpp::Duration(pending->tracking_tolerances.goal_time_tolerance).seconds();
    while (!canceled()) {
      const auto time = now();
      if (time < previous) {error = "ROS time moved backwards during pose execution"; break;}
      const bool advancing = time > previous;
      previous = time;
      const double elapsed = (time - start).seconds();
      if (!readTipPose(*pending, measured, error)) {break;}
      const auto desired = pending->pose_path.sample(elapsed);
      auto velocity = pending->pose_path.sampleVelocity(elapsed);
      if (!advancing) {velocity.setZero();}
      geometry_msgs::msg::TwistStamped feedforward;
      feedforward.header.frame_id = reference_frame_;
      feedforward.header.stamp = time;
      feedforward.twist.linear.x = velocity(0);
      feedforward.twist.linear.y = velocity(1);
      feedforward.twist.linear.z = velocity(2);
      feedforward.twist.angular.x = velocity(3);
      feedforward.twist.angular.y = velocity(4);
      feedforward.twist.angular.z = velocity(5);
      endpoint.target->publish(poseMessage(desired, reference_frame_, time));
      endpoint.feedforward->publish(feedforward);
      if (pending->wants_nominal) {
        msg::RobotNominalReference reference;
        if (!pending->nominal.reference(elapsed, time, reference, error, !advancing)) {
          error = "Nominal sampling failed: " + error; break;
        }
        endpoint.nominal->publish(reference);
        last_nominal_time = elapsed;
      }
      const auto tracking = poseError(desired, measured, pending->goal.world_from_tolerance.rotation());
      const auto active_tracking = poseError(desired, measured,
        pending->goal.world_from_tolerance.rotation(), pending->pose_activation);
      auto output = std::make_shared<Action::Feedback>();
      output->progress = static_cast<float>(0.8 + 0.2 * (duration > 0.0 ? std::clamp(elapsed / duration, 0.0, 1.0) : 1.0));
      output->current_stage = elapsed >= duration ? "converging_pose" : "tracking_pose";
      output->message = "Streaming Cartesian references to " + endpoint.task_id;
      std::copy(tracking.data(), tracking.data() + 6, output->tip_error.begin());
      pending->handle->publish_feedback(output);
      if (elapsed >= duration && within(active_tracking, pending->tracking_tolerances.tip_goal)) {
        success = true; error = "Cartesian pose target reached"; break;
      }
      if (elapsed < duration && !within(active_tracking, pending->tracking_tolerances.tip_path)) {
        error = "Cartesian pose path tracking tolerance violated"; break;
      }
      if (elapsed > duration + allowance) {error = "Cartesian pose goal tracking tolerance timed out"; break;}
      // Wall scheduling remains cancelable when ROS/simulation time is paused.
      next_tick = std::max(next_tick + std::chrono::milliseconds(10), std::chrono::steady_clock::now());
      std::this_thread::sleep_until(next_tick);
    }
    if (canceled()) {success = false; error = "Pose execution canceled or controller deactivated";}
  } catch (const std::exception & exception) {success = false; error = exception.what();}

  if (touched) {
    try {
      publish_zero();
      std::string measured_error;
      // On TF loss, retain the last valid measurement rather than a future target.
      readTipPose(*pending, measured, measured_error);
      endpoint.target->publish(poseMessage(measured, reference_frame_, now()));
      if (pending->wants_nominal) {
        // Stop the stream at rest; synchronized holds keep this last reference.
        msg::RobotNominalReference reference;
        std::string nominal_error;
        if (pending->nominal.reference(last_nominal_time, now(), reference, nominal_error, true)) {
          endpoint.nominal->publish(reference);
        }
      }
      std::string restore_error;
      if (!setAutoUpdate(setter, parameter_name, auto_update, execution_options_.controller_wait_timeout, restore_error)) {
        success = false; error += "; restoring auto_update_target_pose failed: " + restore_error;
      }
    } catch (const std::exception & exception) {
      success = false; error += "; pose cleanup failed: " + std::string(exception.what());
    }
  }
  finish(pending, success, error);
}

}  // namespace sura_manipulation_actions
