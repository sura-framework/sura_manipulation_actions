#pragma once
#include "sura_manipulation_actions/whole_body_nominal.hpp"
#include "sura_manipulation_actions/cartesian_tracking.hpp"
#include "sura_manipulation_actions/cartesian_pose_path.hpp"
#include "sura_manipulation_actions/msg/local_segment.hpp"

namespace sura_manipulation_actions
{
// Geometric interpolation of the original global knots. Only the producing
// optimizer needs the time law that originally associated these configurations.
class ProgressNominal
{
public:
  bool init(
    const trajectory_msgs::msg::JointTrajectory & joints,
    const trajectory_msgs::msg::MultiDOFJointTrajectory & base,
    const std::vector<double> & progress, std::string & error)
  {
    WholeBodyNominal source;
    if (!source.init(joints, base, error)) {return false;}
    if (progress.empty() || progress.size() != joints.points.size() || progress.back() != 1.0 ||
      (progress.front() != 0.0 && progress.front() != 1.0))
    {error = "Incomplete geometric nominal knots"; return false;}
    std::vector<double> coordinates;
    std::vector<WholeBodyNominalSample> samples;
    for (size_t i = 0; i < progress.size(); ++i) {
      if (!std::isfinite(progress[i]) || progress[i] < 0 || progress[i] > 1 ||
        (i && progress[i] < progress[i - 1]))
      {
        error = "Invalid geometric knot order"; return false;
      }
      WholeBodyNominalSample knot;
      if (!source.sample(baseTrajectorySeconds(joints.points[i].time_from_start), knot, error)) {
        return false;
      }
      if (i && progress[i] == progress[i - 1]) {
        if (!knot.pose.isApprox(samples.back().pose, 1e-8) ||
          !knot.joints.isApprox(samples.back().joints, 1e-8))
        {error = "Distinct nominal configurations at identical geometric progress"; return false;}
        continue;
      }
      coordinates.push_back(progress[i]);
      samples.push_back(std::move(knot));
    }
    progress_ = std::move(coordinates); knots_ = std::move(samples);
    return true;
  }

  void unwrapContinuous(const std::vector<size_t> & columns)
  {
    for (size_t i = 1; i < knots_.size(); ++i) {
      for (size_t j : columns) {
        const double before = knots_[i - 1].joints(j);
        knots_[i].joints(j) = before +
          std::remainder(knots_[i].joints(j) - before, 2 * std::acos(-1.0));
      }
    }
  }

  WholeBodyNominalSample sample(double s, double rate) const
  {
    const size_t last = progress_.size() - 1;
    size_t b = std::upper_bound(progress_.begin(), progress_.end(), s) - progress_.begin();
    if (rate < 0) {b = std::lower_bound(progress_.begin(), progress_.end(), s) - progress_.begin();}
    b = last ? std::clamp(b, size_t(1), last) : 0;
    const auto & left = knots_[b ? b - 1 : b];
    const auto & right = knots_[b];
    const double span = last ? progress_[b] - progress_[b - 1] : 1.0;
    const double u = last ? std::clamp((s - progress_[b - 1]) / span, 0.0, 1.0) : 0.0;
    WholeBodyNominalSample out;
    out.pose.translation() = (1 - u) * left.pose.translation() + u * right.pose.translation();
    const Eigen::Quaterniond a(left.pose.rotation()), z(right.pose.rotation());
    out.pose.linear() = a.slerp(u, z).normalized().toRotationMatrix();
    out.base = baseCoordinates(out.pose);
    const auto velocity = (rate / span * cartesianTrackingError(
        right.pose, left.pose, Eigen::Matrix3d::Identity())).eval();
    out.twist = baseTwist(velocity.head<3>(), velocity.tail<3>());
    out.joints = (1 - u) * left.joints + u * right.joints;
    out.joint_velocities = (rate / span) * (right.joints - left.joints);
    out.base_rates.head<3>() = velocity.head<3>();
    out.base_rates.tail<3>() =
      baseAngularVelocityMap(out.base).fullPivLu().solve(velocity.tail<3>());
    return out;
  }

private:
  std::vector<double> progress_;
  std::vector<WholeBodyNominalSample> knots_;
};

struct PathPhase {double s; double rate;};

// Progress between local time knots. A hold has a zero tangent and a retreat a negative one.
inline PathPhase segmentPhase(const msg::LocalSegment & segment, double elapsed)
{
  const auto & p = segment.joints.points;
  const auto & s = segment.progress;
  if (elapsed >= baseTrajectorySeconds(p.back().time_from_start)) {return {s.back(), 0};}
  auto it = std::upper_bound(
    p.begin(), p.end(), std::max(0.0, elapsed),
    [](double t, const auto & point) {return t < baseTrajectorySeconds(point.time_from_start);});
  size_t b = std::clamp<size_t>(it - p.begin(), 1, p.size() - 1);
  double t = baseTrajectorySeconds(p[b - 1].time_from_start);
  double rate = (s[b] - s[b - 1]) / (baseTrajectorySeconds(p[b].time_from_start) - t);
  return {s[b - 1] + rate * (std::max(0.0, elapsed) - t), rate};
}

// Local projection in the same task axes and tolerance metric used for tracking.
// Unobservable translation/rotation contributes no evidence of progress.
inline double measuredProgress(
  const CartesianPosePath & path, double previous,
  const Eigen::Isometry3d & measured, const Eigen::Matrix3d & axes,
  const BaseCoordinates & activation, const BaseCoordinates & tolerances, double window)
{
  const auto residual =
    cartesianTrackingError(measured, path.atProgress(previous), axes, activation);
  const BaseCoordinates tangent = path.progressTangent().cwiseProduct(activation);
  BaseCoordinates direction;
  direction << axes.transpose() * tangent.head<3>(), axes.transpose() * tangent.tail<3>();
  const auto scale = tolerances.cwiseMax(1e-6).cwiseInverse().eval();
  direction = direction.cwiseProduct(scale);
  if (direction.squaredNorm() < 1e-14) {return previous;}
  const double change = direction.dot(residual.cwiseProduct(scale)) / direction.squaredNorm();
  return std::clamp(previous + std::clamp(change, -window, window), 0.0, 1.0);
}
}  // namespace sura_manipulation_actions
