#include "sura_manipulation_actions/plan_cartesian_trajectory_action_server.hpp"
#include "sura_manipulation_actions/cartesian_tracking.hpp"
#include "sura_manipulation_actions/progress_nominal.hpp"

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

visualization_msgs::msg::MarkerArray exploredMarkers(const std_msgs::msg::Header & header,
  const std::vector<msg::LocalCandidatePath> & paths = {})
{
  using Marker = visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray markers;
  auto & clear = markers.markers.emplace_back();
  clear.header = header;
  clear.action = Marker::DELETEALL;
  for (const auto & path : paths) {
    if (path.poses.empty()) {continue;}
    auto & marker = markers.markers.emplace_back();
    marker.header = header;
    marker.ns = "explored_paths";
    marker.id = static_cast<int32_t>(markers.markers.size() - 1);
    marker.type = path.poses.size() == 1 ? Marker::SPHERE_LIST : Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.003;
    marker.color.r = path.rejoined ? 0.15F : 1.0F;
    marker.color.g = path.rejoined ? 0.8F : 0.55F;
    marker.color.b = path.rejoined ? 1.0F : 0.1F;
    marker.color.a = 0.85F;
    marker.points.reserve(path.poses.size());
    for (const auto & pose : path.poses) {marker.points.push_back(pose.position);}
  }
  return markers;
}

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
    if (pending.use_tip_offset) {
      // The pose task controls the offset point: every pose in this backend is a tip pose.
      Eigen::Isometry3d offset;
      if (!readTipOffset(pending, offset, error)) {return false;}
      pose = pose * offset;
    }
    return true;
  } catch (const std::exception & exception) {
    error = "Tool TF unavailable: " + std::string(exception.what()); return false;
  }
}

bool PlanCartesianTrajectoryActionServer::readTipOffset(
  const PendingPlan & pending, Eigen::Isometry3d & offset, std::string & error) const
{
  const auto & endpoint = pose_endpoints_.at(pending.arm.name);
  std::scoped_lock lock(endpoint.offset->mutex);
  if (!endpoint.offset->valid) {
    error = "No tip offset received on " + endpoint.offset_topic + " (use_offset is on)"; return false;
  }
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - endpoint.offset->received).count();
  if (age > pose_offset_max_age_) {
    error = "Tip offset on " + endpoint.offset_topic + " is stale"; return false;
  }
  offset = endpoint.offset->pose;
  return true;
}

bool PlanCartesianTrajectoryActionServer::readUseOffset(
  const PendingPlan & pending, bool & use, double timeout, std::string & error)
{
  const auto & endpoint = pose_endpoints_.at(pending.arm.name);
  const auto & client = endpoint.parameters;
  auto input = std::make_shared<GetParameters::Request>();
  input->names = {"tasks." + endpoint.task_id + ".use_offset"};
  const auto response = request<GetParameters>(client, input, timeout, error,
    [this, &pending]() {return stopped_.load() || pending.handle->is_canceling();});
  if (!response) {return false;}
  using Type = rcl_interfaces::msg::ParameterType;
  if (response->values.size() != 1) {error = "use_offset parameter request returned no value"; return false;}
  const auto & value = response->values[0];
  if (value.type == Type::PARAMETER_BOOL) {use = value.bool_value; return true;}
  // A controller without offset support does not declare the parameter.
  if (value.type == Type::PARAMETER_NOT_SET) {use = false; return true;}
  error = "use_offset must be a boolean parameter"; return false;
}

void PlanCartesianTrajectoryActionServer::planPoseGoal(const std::shared_ptr<PendingPlan> & pending)
{
  try {
    std::string error;
    const auto canceled = [this, &pending]() {
        return stopped_.load() || pending->handle->is_canceling() || !pending->handle->is_active();
      };
    feedback(pending->handle, 0.0F, "acquiring_pose", "Reading tool TF; generating a geometric pose reference");
    {
      // With use_offset on the pose task, start pose and target are poses of the tip, not of the tool.
      // Without a reachable controller at planning time the plan is for the tool; execution rechecks.
      std::string offset_error;
      bool use_offset = false;
      if (!readUseOffset(*pending, use_offset, std::min(execution_options_.controller_wait_timeout, 1.0), offset_error)) {
        if (pending->use_predictive_local_planner) {finish(pending,false,offset_error); return;}
        RCLCPP_WARN(get_logger(), "Cannot read use_offset of %s; planning for the tool: %s",
          pose_endpoints_.at(pending->arm.name).task_id.c_str(), offset_error.c_str());
      }
      if (pending->use_predictive_local_planner && require_tip_offset_ && !use_offset) {
        finish(pending,false,"Predictive zip-tie tracking requires the selected task to use its measured tip offset");
        return;
      }
      pending->use_tip_offset = use_offset;
      if (use_offset) {
        feedback(pending->handle, 0.0F, "acquiring_pose", "use_offset is on: planning the pose of the tip");
      }
    }
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
  {
    bool use_offset_now = false;
    if (!readUseOffset(pending, use_offset_now, execution_options_.controller_wait_timeout, error)) {return false;}
    if (use_offset_now != pending.use_tip_offset) {
      error = std::string("use_offset of the pose task changed since planning (") +
        (pending.use_tip_offset ? "planned for the tip" : "planned for the tool") + "); send a new goal";
      return false;
    }
  }
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
    pending.nominal_progress = wrapped.result->nominal_progress;
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

bool PlanCartesianTrajectoryActionServer::executePredictive(const PendingPlan & pending,
  msg::RobotNominalReference & last, std::string & error)
{
  const auto & endpoint = pose_endpoints_.at(pending.arm.name);
  const auto & path = pending.pose_path;
  ProgressNominal global;
  if (!global.init(pending.nominal_joints,pending.nominal_base,pending.nominal_progress,error)) {return false;}
  CheckAction::Goal request_template;
  setTrackingGoal(request_template,pending,reference_frame_,endpoint.task_id);
  request_template.nominal_joints = pending.nominal_joints;
  request_template.nominal_base = pending.nominal_base;
  request_template.prediction = prediction_settings_;
  request_template.prediction.global_progress = pending.nominal_progress;
  if (!moveit_node_->get_parameter("robot_description",request_template.prediction.robot_description) ||
    !moveit_node_->get_parameter("robot_description_semantic",request_template.prediction.robot_description_semantic))
  {error = "Local prediction needs the planning URDF and SRDF"; return false;}

  std::atomic_bool stop{false};
  const auto canceled = [&] {
      return stop.load() || !rclcpp::ok() || stopped_.load() || !execution_active_.load() ||
        pending.handle->is_canceling() || !pending.handle->is_active();
    };
  struct Reply {
    CheckAction::Result report;
    double requested = 0;
    bool at_boundary = false;
    bool ok = false;
    std::string reason;
  };
  std::future<Reply> worker;
  struct CancelOnExit {
    std::atomic_bool & flag;
    ~CancelOnExit() {flag.store(true);}
  } cancel_on_exit{stop};  // Cancel the remote goal before waiting for the future.
  const auto clock = std::chrono::steady_clock::now;
  auto next_request = clock();
  auto previous = now();
  double s = path.duration() == 0 ? 1.0 : 0.0;
  double confirmed = s, maximum = s, local = 0.0;
  double no_progress = 0.0;
  msg::LocalSegment active;
  WholeBodyNominal sampler;
  const Eigen::Matrix3d axes = pending.goal.world_from_tolerance.rotation();
  const auto tolerance = Eigen::Map<const BaseCoordinates>(pending.tracking_tolerances.tip_path.data());
  auto fail = [&](const std::string & reason) {
      error = reason;
      // Hold the measured whole body when available. Retaining a future
      // configuration here could keep driving the synchronized arm holds.
      planning_scene::PlanningScenePtr scene;
      std::string unavailable;
      if (context_->snapshot(pending.nominal_joints.joint_names,0.02,state_max_age_,scene,unavailable,canceled)) {
        const auto & state = scene->getCurrentState();
        last.header.frame_id = reference_frame_;
        last.base_pose = poseMessage(state.getGlobalLinkTransform(base_frame_),reference_frame_,now()).pose;
        last.joint_names = pending.nominal_joints.joint_names;
        last.positions.clear();
        for (const auto & joint : last.joint_names) {last.positions.push_back(state.getVariablePosition(joint));}
        last.velocities.assign(last.positions.size(),0.0);
        last.base_twist = geometry_msgs::msg::Twist();
      }
      feedback(pending.handle,static_cast<float>(s),"local_replan_required",error);
      return false;
    };
  while (!canceled()) {
    const auto stamp = now();
    const double dt = (stamp-previous).seconds();
    if (dt < 0) {return fail("Local prediction invalidated by backward ROS time jump");}
    previous = stamp;
    Eigen::Isometry3d measured;
    if (!readTipPose(pending,measured,error)) {return fail(error);}
    confirmed = measuredProgress(path,confirmed,measured,axes,pending.pose_activation,tolerance,1.0);
    const auto tracking = poseError(path.atProgress(s),measured,axes,pending.pose_activation);
    if (!within(tracking,pending.tracking_tolerances.tip_path)) {return fail("Measured tip left the Cartesian tracking tube");}
    const bool boundary = sampler.empty() || local >= sampler.duration()-1e-9;
    if (worker.valid() && worker.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto reply = worker.get();
      if (!reply.report.explored_paths.empty()) {
        std_msgs::msg::Header header;
        header.frame_id = reference_frame_;
        header.stamp = stamp;
        local_path_pub_->publish(exploredMarkers(header, reply.report.explored_paths));
      }
      if (reply.ok) {
        const auto & report = reply.report;
        WholeBodyNominal candidate;
        if (report.segment.progress.size() < 2 || report.segment.progress.size() != report.segment.joints.points.size() ||
          !candidate.init(report.segment.joints,report.segment.base,error)) {return fail("Invalid local segment: "+error);}
        const bool improvement = !std::isfinite(report.continuation_cost) ||
          report.segment_cost < report.continuation_cost-prediction_settings_.improvement;
        bool continuous = std::abs(s-report.segment.progress.front()) <= 1e-6;
        WholeBodyNominalSample first;
        if (!candidate.sample(0.0,first,error)) {return fail(error);}
        if (!last.positions.empty()) {
          const auto prev_joints = Eigen::Map<const Eigen::VectorXd>(last.positions.data(),last.positions.size());
          const auto & p = last.base_pose.position;
          const auto & q = last.base_pose.orientation;
          continuous = continuous && first.joints.size() == prev_joints.size() &&
            (first.joints-prev_joints).lpNorm<Eigen::Infinity>() <= start_joint_tolerance_ &&
            (first.pose.translation()-Eigen::Vector3d(p.x,p.y,p.z)).norm() <= base_position_tolerance_ &&
            Eigen::Quaterniond(first.pose.rotation()).angularDistance(Eigen::Quaterniond(q.w,q.x,q.y,q.z)) <= base_orientation_tolerance_;
        }
        std::ostringstream log;
        log << "requested=" << reply.requested << " confirmed=" << confirmed << " cost=" << report.segment_cost
            << " incumbent=" << report.continuation_cost << " rank=" << report.hierarchy_rank
            << " configuration_error=" << report.configuration_error << " clearance=" << report.minimum_clearance
            << " candidates=" << report.candidates << " compute_s=" << report.compute_seconds << " constraints=";
        for (const auto & id : report.active_constraints) {log << id << ',';}
        bool accepted = continuous && (boundary || improvement);
        if (accepted) {
          active = report.segment; sampler = std::move(candidate); local = 0;
          nav_msgs::msg::Path base_path;
          base_path.header.frame_id = reference_frame_;
          base_path.header.stamp = stamp;
          for (size_t i = 0; i < active.progress.size(); ++i) {
            base_path.poses.push_back(poseMessage(
              transformPose(active.base.points[i].transforms.front()), reference_frame_, stamp));
          }
          local_base_path_pub_->publish(base_path);
          nav_msgs::msg::Path predicted_path;
          predicted_path.header = base_path.header;
          for (const auto & sample : report.trace) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = predicted_path.header;
            pose.pose = sample.tip_pose;
            predicted_path.poses.push_back(pose);
          }
          predicted_path_pub_->publish(predicted_path);
        }
        feedback(pending.handle,static_cast<float>(s),accepted ? "local_segment_accepted" : "local_segment_retained",log.str());
        if (boundary && reply.at_boundary && !continuous) {return fail("Local segment cannot reconnect continuously");}
      } else {
        if ((reply.at_boundary && boundary) || reply.reason.find("STALE_SCENE") == 0 ||
          reply.reason.find("NO_LOCAL_CONTINUATION") == 0) {return fail(reply.reason);}
        // A validated prefix may finish while a prediction based on an older
        // moving state is in flight. Retry once from its actual endpoint.
        feedback(pending.handle,static_cast<float>(s),"local_prediction_rejected",reply.reason);
      }
      next_request = clock() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(prediction_period_));
    }
    const bool exhausted = sampler.empty() || local >= sampler.duration()-1e-9;
    if (!worker.valid() && (exhausted || clock() >= next_request)) {
      auto goal = request_template;
      goal.prediction.incumbent = active;
      goal.prediction.incumbent_elapsed = local;
      goal.prediction.start_progress = s;
      goal.prediction.maximum_progress = maximum;
      worker = std::async(std::launch::async,[&,clock,goal=std::move(goal),exhausted]() mutable {
        Reply reply; reply.requested = goal.prediction.start_progress; reply.at_boundary = exhausted;
        planning_scene::PlanningScenePtr before, after;
        auto snapshot = [&](auto & scene) {
            return context_->snapshot(pending.nominal_joints.joint_names,state_timeout_,state_max_age_,scene,reply.reason,canceled);
          };
        if (!snapshot(before)) {return reply;}
        before->getPlanningSceneMsg(goal.prediction.scene);
        const auto deadline = clock()+std::chrono::duration<double>(stpik_timeout_);
        CheckHandle::WrappedResult result;
        if (!controllerRequest<CheckAction>(endpoint.checker,goal,deadline,canceled,result,reply.reason)) {return reply;}
        if (result.result) {reply.report = std::move(*result.result);}
        if (!result.result || result.code != rclcpp_action::ResultCode::SUCCEEDED ||
          reply.report.outcome != CheckAction::Result::SUCCESS || !reply.report.validation_id) {
          reply.reason = result.result ? reply.report.code+": "+reply.report.message : "Missing local prediction";
          return reply;
        }
        if (!snapshot(after)) {return reply;}
        moveit_msgs::msg::PlanningScene fresh;
        after->getPlanningSceneMsg(fresh);
        const auto & original = goal.prediction.scene;
        if (original.world != fresh.world || original.allowed_collision_matrix != fresh.allowed_collision_matrix ||
          original.fixed_frame_transforms != fresh.fixed_frame_transforms || original.link_padding != fresh.link_padding ||
          original.link_scale != fresh.link_scale ||
          original.robot_state.attached_collision_objects != fresh.robot_state.attached_collision_objects) {
          reply.reason = "STALE_SCENE: collision geometry changed"; return reply;
        }
        const auto & joint_names = pending.nominal_joints.joint_names;
        for (const auto * joint : after->getRobotModel()->getActiveJointModels()) {
          if (joint->getVariableCount() != 1) {continue;}
          const auto & name = joint->getVariableNames().front();
          if (std::find(joint_names.begin(),joint_names.end(),name) == joint_names.end() &&
            std::abs(after->getCurrentState().getVariablePosition(name) -
            before->getCurrentState().getVariablePosition(name)) > start_joint_tolerance_) {
            reply.reason = "STALE_SCENE: passive joint changed"; return reply;
          }
        }
        goal.validation_id = reply.report.validation_id;
        if (!controllerRequest<CheckAction>(endpoint.checker,goal,deadline,canceled,result,reply.reason)) {return reply;}
        reply.ok = result.result && result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          result.result->outcome == CheckAction::Result::SUCCESS && result.result->validation_id == goal.validation_id;
        if (!reply.ok) {reply.reason = "STALE_SNAPSHOT: local prediction no longer valid";}
        return reply;
      });
    }
    if (!sampler.empty()) {
      const double next = std::min(sampler.duration(),local+dt);
      const auto phase = segmentPhase(active,next);
      const auto desired = path.atProgress(phase.s);
      const bool allowed = within(poseError(desired,measured,axes,pending.pose_activation),pending.tracking_tolerances.tip_path) &&
        within(poseError(desired,path.atProgress(confirmed),axes,pending.pose_activation),pending.tracking_tolerances.tip_path);
      if (!allowed) {return fail("Validated progress no longer agrees with the measured tip");}
      const bool moving = dt > 0 && next > local;
      local = next; s = phase.s;
      double gained = s-maximum;
      maximum = std::max(maximum,s);
      no_progress = gained > 1e-8 ? 0.0 : no_progress+dt;
      const auto ref = poseMessage(desired,reference_frame_,stamp);
      geometry_msgs::msg::TwistStamped ff; ff.header = ref.header;
      const BaseCoordinates velocity = path.progressTangent()*(moving ? phase.rate : 0.0);
      ff.twist = baseTwist(velocity.head<3>(),velocity.tail<3>());
      if (!sampler.reference(local,stamp,last,error,!moving)) {return fail(error);}
      endpoint.target->publish(ref); endpoint.feedforward->publish(ff); endpoint.nominal->publish(last);
      auto update = std::make_shared<Action::Feedback>();
      update->current_stage = "local_tracking";
      update->progress = static_cast<float>(s);
      std::copy_n(tracking.data(),6,update->tip_error.begin());
      pending.handle->publish_feedback(update);
      if (s >= 1.0-1e-9 && within(poseError(path.atProgress(1),measured,axes,pending.pose_activation),
        pending.tracking_tolerances.tip_goal)) {
        const auto nominal = global.sample(1,0);
        planning_scene::PlanningScenePtr current;
        if (!context_->snapshot(pending.nominal_joints.joint_names,state_timeout_,state_max_age_,current,error,canceled))
        {return fail(error);}
        const auto & state = current->getCurrentState();
        const auto base_error = cartesianTrackingError(nominal.pose,state.getGlobalLinkTransform(base_frame_),Eigen::Matrix3d::Identity());
        double squared = base_error.head<3>().squaredNorm()/std::pow(prediction_settings_.position_scale,2) +
          base_error.tail<3>().squaredNorm()/std::pow(prediction_settings_.angle_scale,2);
        for (size_t i = 0; i < pending.nominal_joints.joint_names.size(); ++i) {
          squared += std::pow((state.getVariablePosition(pending.nominal_joints.joint_names[i])-nominal.joints(i))/
            prediction_settings_.angle_scale,2);
        }
        if (std::sqrt(squared) <= prediction_settings_.recovery_threshold) {
          error = "Measured tip and whole body reached global endpoint"; return true;
        }
      }
      if (no_progress > stpik_timeout_) {return fail("No sustained local recovery; global replanning required");}
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  error = "Predictive execution canceled"; return false;
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
  msg::RobotNominalReference last_predictive_reference;
  const bool predictive = pending->use_predictive_local_planner;
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
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = reference_frame_;
    empty_path.header.stamp = now();
    local_path_pub_->publish(exploredMarkers(empty_path.header));
    local_base_path_pub_->publish(empty_path);
    predicted_path_pub_->publish(empty_path);
    if (predictive) {
      success = executePredictive(*pending,last_predictive_reference,error);
    } else {
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
        if (predictive && !last_predictive_reference.positions.empty()) {
          last_predictive_reference.base_twist = geometry_msgs::msg::Twist();
          std::fill(last_predictive_reference.velocities.begin(),last_predictive_reference.velocities.end(),0.0);
          last_predictive_reference.header.stamp = now();
          endpoint.nominal->publish(last_predictive_reference);
        } else if (!predictive && pending->nominal.reference(last_nominal_time, now(), reference, nominal_error, true)) {
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
