#pragma once

#include "sura_manipulation_actions/base_trajectory_sampling.hpp"
#include "sura_manipulation_actions/msg/robot_nominal_reference.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace sura_manipulation_actions
{

// One sample of a whole-body nominal: base pose (ZYX Euler coordinates and their
// rates) plus every listed joint. The same interpolation is used by the planner
// (streaming), the offline STPIK simulation and the trajectory optimizer.
struct WholeBodyNominalSample
{
  BaseCoordinates base = BaseCoordinates::Zero();
  BaseCoordinates base_rates = BaseCoordinates::Zero();
  Eigen::VectorXd joints;
  Eigen::VectorXd joint_velocities;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  geometry_msgs::msg::Twist twist;
};

class WholeBodyNominal
{
public:
  bool empty() const {return times_.empty();}
  double duration() const {return times_.empty() ? 0.0 : times_.back();}
  const std::vector<std::string> & joint_names() const {return names_;}
  const std::string & frame() const {return frame_;}

  bool init(
    const trajectory_msgs::msg::JointTrajectory & joints,
    const trajectory_msgs::msg::MultiDOFJointTrajectory & base, std::string & error)
  {
    *this = WholeBodyNominal();
    if (joints.points.empty() || base.points.empty() || joints.joint_names.empty()) {
      error = "Nominal trajectory needs base points, joint names and joint points"; return false;
    }
    if (joints.points.size() != base.points.size()) {
      error = "Nominal base and joint waypoint counts differ"; return false;
    }
    if (!prepareBaseTrajectory(base, base_knots_, error)) {return false;}
    const auto count = joints.joint_names.size();
    for (size_t i = 0; i < joints.points.size(); ++i) {
      const auto & point = joints.points[i];
      if (point.time_from_start != base.points[i].time_from_start) {
        error = "Nominal base and joint waypoint times differ"; *this = WholeBodyNominal(); return false;
      }
      if (point.positions.size() != count || (!point.velocities.empty() && point.velocities.size() != count) ||
        (!point.accelerations.empty() && point.accelerations.size() != count) ||
        (!point.accelerations.empty() && point.velocities.empty()) ||
        (i && (point.velocities.empty() != joints.points.front().velocities.empty() ||
        point.accelerations.empty() != joints.points.front().accelerations.empty())))
      {error = "Nominal joint point dimensions are inconsistent"; *this = WholeBodyNominal(); return false;}
      Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(point.positions.data(), count);
      Eigen::VectorXd v = point.velocities.empty() ? Eigen::VectorXd::Zero(count) :
        Eigen::VectorXd(Eigen::Map<const Eigen::VectorXd>(point.velocities.data(), count));
      Eigen::VectorXd a = point.accelerations.empty() ? Eigen::VectorXd::Zero(count) :
        Eigen::VectorXd(Eigen::Map<const Eigen::VectorXd>(point.accelerations.data(), count));
      if (!q.allFinite() || !v.allFinite() || !a.allFinite()) {
        error = "Nominal joint values must be finite"; *this = WholeBodyNominal(); return false;
      }
      times_.push_back(base_knots_[i].time);
      positions_.push_back(std::move(q)); velocities_.push_back(std::move(v));
      accelerations_.push_back(std::move(a));
    }
    if (std::abs(times_.front()) > 1e-12) {
      error = "Nominal trajectory must start at time_from_start zero"; *this = WholeBodyNominal(); return false;
    }
    has_velocity_ = !joints.points.front().velocities.empty();
    has_acceleration_ = !joints.points.front().accelerations.empty();
    names_ = joints.joint_names;
    frame_ = base.header.frame_id;
    return true;
  }

  bool sample(double elapsed, WholeBodyNominalSample & out, std::string & error) const
  {
    if (empty() || !std::isfinite(elapsed)) {error = "Cannot sample an empty nominal trajectory"; return false;}
    BaseTrajectorySample base;
    if (!sampleBaseTrajectory(base_knots_, std::max(0.0, elapsed), base, error)) {return false;}
    out.base = base.coordinates; out.base_rates = base.velocity; out.pose = base.pose; out.twist = base.twist;
    const auto count = static_cast<Eigen::Index>(names_.size());
    out.joints.resize(count); out.joint_velocities.resize(count);
    if (elapsed <= times_.front() || elapsed >= times_.back()) {
      const size_t index = elapsed <= times_.front() ? 0 : times_.size() - 1;
      out.joints = positions_[index];
      const bool outside = elapsed < times_.front() || elapsed > times_.back();
      out.joint_velocities = outside ? Eigen::VectorXd::Zero(count) : velocities_[index];
      return true;
    }
    const auto after = std::upper_bound(times_.begin(), times_.end(), elapsed);
    const size_t b = static_cast<size_t>(after - times_.begin()), a = b - 1;
    const double span = times_[b] - times_[a], u = (elapsed - times_[a]) / span;
    for (Eigen::Index j = 0; j < count; ++j) {
      const auto c = baseHermitePolynomial(positions_[a](j), positions_[b](j),
        velocities_[a](j), velocities_[b](j), accelerations_[a](j), accelerations_[b](j),
        span, has_velocity_, has_acceleration_);
      out.joints(j) = evaluateBasePolynomial(c, u);
      out.joint_velocities(j) = evaluateBasePolynomial(c, u, 1) / span;
    }
    if (!out.joints.allFinite() || !out.joint_velocities.allFinite()) {
      error = "Nominal joint interpolation is not finite"; return false;
    }
    return true;
  }

  // Reference message for the robot_nominal task. Outside the timed interval the
  // velocities are zero, so the last sample becomes a steady hold.
  bool reference(double elapsed, const builtin_interfaces::msg::Time & stamp,
    msg::RobotNominalReference & out, std::string & error, bool zero_velocity = false) const
  {
    WholeBodyNominalSample value;
    if (!sample(elapsed, value, error)) {return false;}
    out = msg::RobotNominalReference();
    out.header.frame_id = frame_; out.header.stamp = stamp;
    out.base_pose.position.x = value.pose.translation().x();
    out.base_pose.position.y = value.pose.translation().y();
    out.base_pose.position.z = value.pose.translation().z();
    const Eigen::Quaterniond q(value.pose.rotation());
    out.base_pose.orientation.x = q.x(); out.base_pose.orientation.y = q.y();
    out.base_pose.orientation.z = q.z(); out.base_pose.orientation.w = q.w();
    if (!zero_velocity) {out.base_twist = value.twist;}
    out.joint_names = names_;
    out.positions.assign(value.joints.data(), value.joints.data() + value.joints.size());
    out.velocities.assign(static_cast<size_t>(value.joints.size()), 0.0);
    if (!zero_velocity) {
      out.velocities.assign(value.joint_velocities.data(),
        value.joint_velocities.data() + value.joint_velocities.size());
    }
    return true;
  }

private:
  std::vector<BaseTrajectoryKnot> base_knots_;
  std::vector<double> times_;
  std::vector<Eigen::VectorXd> positions_, velocities_, accelerations_;
  std::vector<std::string> names_;
  std::string frame_;
  bool has_velocity_ = false;
  bool has_acceleration_ = false;
};

inline builtin_interfaces::msg::Duration nominalDuration(double seconds)
{
  builtin_interfaces::msg::Duration out;
  const double whole = std::floor(seconds);
  out.sec = static_cast<int32_t>(whole);
  out.nanosec = static_cast<uint32_t>(std::min(999999999.0, std::round((seconds - whole) * 1e9)));
  return out;
}

// Build rest-to-rest messages from knots. Interior velocities use central differences
// (cubic Hermite, C1); the endpoints are at rest like the Cartesian tool profile.
inline bool buildWholeBodyNominal(
  const std::vector<double> & times, const std::vector<BaseCoordinates> & base,
  const std::vector<Eigen::VectorXd> & joints, const std::vector<std::string> & names,
  const std::string & frame, trajectory_msgs::msg::JointTrajectory & joint_out,
  trajectory_msgs::msg::MultiDOFJointTrajectory & base_out, std::string & error)
{
  const size_t n = times.size();
  if (n < 2 || base.size() != n || joints.size() != n || names.empty()) {
    error = "Nominal trajectory needs at least two consistent knots"; return false;
  }
  joint_out = trajectory_msgs::msg::JointTrajectory();
  base_out = trajectory_msgs::msg::MultiDOFJointTrajectory();
  joint_out.joint_names = names;
  base_out.header.frame_id = frame;
  joint_out.header.frame_id = frame;
  base_out.joint_names = {"floating_base"};
  for (size_t k = 0; k < n; ++k) {
    if (static_cast<size_t>(joints[k].size()) != names.size() || !joints[k].allFinite() ||
      !base[k].allFinite() || !std::isfinite(times[k]) || (k && times[k] <= times[k - 1]))
    {error = "Nominal knots must be finite with increasing times"; return false;}
    BaseCoordinates rates = BaseCoordinates::Zero();
    Eigen::VectorXd velocity = Eigen::VectorXd::Zero(joints[k].size());
    if (k > 0 && k + 1 < n) {
      const double span = times[k + 1] - times[k - 1];
      rates = (base[k + 1] - base[k - 1]) / span;
      velocity = (joints[k + 1] - joints[k - 1]) / span;
    }
    auto point = baseTrajectoryPoint(base[k], rates, BaseCoordinates::Zero(), nominalDuration(times[k]));
    point.accelerations.clear();
    base_out.points.push_back(point);
    trajectory_msgs::msg::JointTrajectoryPoint joint;
    joint.positions.assign(joints[k].data(), joints[k].data() + joints[k].size());
    joint.velocities.assign(velocity.data(), velocity.data() + velocity.size());
    joint.time_from_start = point.time_from_start;
    joint_out.points.push_back(joint);
  }
  // Rounding to nanoseconds must not collapse two knots.
  for (size_t k = 1; k < n; ++k) {
    if (baseTrajectorySeconds(base_out.points[k].time_from_start) <=
      baseTrajectorySeconds(base_out.points[k - 1].time_from_start))
    {error = "Nominal knot times are too close"; return false;}
  }
  return true;
}

}  // namespace sura_manipulation_actions
