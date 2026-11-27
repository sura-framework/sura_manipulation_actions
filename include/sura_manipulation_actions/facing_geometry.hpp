#pragma once

// Geometry of the "end effectors facing" task, shared by the bimanual planner, the
// task-priority EndEffectorsFacingTask, the STPIK check and the facing executor.
//
// Rows (fixed order, 6 DOF):
//   0-1  left reference pointing axis (left EE frame) aligned with +d (towards the right EE)
//   2-3  right reference pointing axis (right EE frame) aligned with -d (towards the left EE)
//   4    relative roll around d (left -> right)
//   5    distance between both EE
// where d = (p_right - p_left) / ||p_right - p_left||.
//
// The reference pointing axes are arbitrary unit vectors. The zero of the roll is always
// attached to the pure signed goal axes (+-x -> +-y, +-y -> +-z, +-z -> +-x).

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace sura_manipulation_actions::facing
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kPureAxisTolerance = 1e-3;
constexpr double kAlignedEpsilon = 1e-9;
// x^2 + y^2 of the roll atan2 equals |proj(b_left)|^2 * |proj(b_right)|^2.
constexpr double kRollMinSquaredNorm = 1e-4;

using Vector6 = Eigen::Matrix<double, 6, 1>;

inline double wrapAngle(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}

struct PureAxis
{
  int index = 2;
  double sign = 1.0;
  Eigen::Vector3d vector() const {return sign * Eigen::Vector3d::Unit(index);}
};

// Returns false (and fills message) if axis is not a pure signed axis.
inline bool pureAxis(const Eigen::Vector3d & axis, PureAxis & out, std::string & message)
{
  if (!axis.allFinite()) {
    message = "pointing axis must be finite";
    return false;
  }
  Eigen::Index max_index = 0;
  axis.cwiseAbs().maxCoeff(&max_index);
  for (Eigen::Index i = 0; i < 3; ++i) {
    const double expected = i == max_index ? 1.0 : 0.0;
    if (std::abs(std::abs(axis(i)) - expected) > kPureAxisTolerance) {
      std::ostringstream stream;
      stream << "pointing axis [" << axis.transpose() << "] must be a pure signed axis (+-x, +-y or +-z)";
      message = stream.str();
      return false;
    }
  }
  out.index = static_cast<int>(max_index);
  out.sign = axis(max_index) > 0.0 ? 1.0 : -1.0;
  return true;
}

// World-frame reference axis of an EE for the roll zero (cyclic: x->y, y->z, z->x).
inline Eigen::Vector3d rollReferenceAxis(const Eigen::Matrix3d & rotation, const PureAxis & axis)
{
  return axis.sign * rotation.col((axis.index + 1) % 3);
}

struct RollGeometry
{
  bool valid = false;
  double roll = 0.0;
  double x = 0.0;
  double y = 0.0;
};

inline RollGeometry rollGeometry(
  const Eigen::Vector3d & d, const Eigen::Vector3d & b_left, const Eigen::Vector3d & b_right)
{
  RollGeometry out;
  out.y = d.dot(b_left.cross(b_right));
  out.x = b_left.dot(b_right) - d.dot(b_left) * d.dot(b_right);
  out.valid = out.x * out.x + out.y * out.y > kRollMinSquaredNorm;
  out.roll = out.valid ? std::atan2(out.y, out.x) : 0.0;
  return out;
}

// Orthonormal pair perpendicular to the unit axis, in the same (EE) frame. For a pure
// axis it is the cyclic pair (e_{i+1}, e_{i+2}).
inline void pointingBasis(const Eigen::Vector3d & axis, Eigen::Vector3d & u1, Eigen::Vector3d & u2)
{
  Eigen::Index i = 0;
  axis.cwiseAbs().maxCoeff(&i);
  const Eigen::Vector3d seed = Eigen::Vector3d::Unit((i + 1) % 3);
  u1 = (seed - seed.dot(axis) * axis).normalized();
  u2 = (axis(i) >= 0.0 ? 1.0 : -1.0) * axis.cross(u1);
}

struct Reference
{
  // Unit reference pointing axes, each expressed in its own EE frame.
  Eigen::Vector3d left_axis = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d right_axis = Eigen::Vector3d::UnitZ();
  double roll = 0.0;
  double distance = 0.0;
};

// Derivative of a Reference (per second, or per unit of progress).
struct ReferenceRate
{
  Eigen::Vector3d left_axis = Eigen::Vector3d::Zero();
  Eigen::Vector3d right_axis = Eigen::Vector3d::Zero();
  double roll = 0.0;
  double distance = 0.0;
};

// Task-space velocity (same rows and signs as the task jacobian) that follows a moving reference.
inline Vector6 feedforwardRows(const Reference & reference, const ReferenceRate & rate)
{
  Vector6 out;
  const auto pointing = [&out](const Eigen::Vector3d & axis, const Eigen::Vector3d & axis_rate, int row) {
      Eigen::Vector3d u1, u2;
      pointingBasis(axis, u1, u2);
      // omega_ee - omega_line = -R (a x a_dot) keeps R a on the line while a moves.
      const Eigen::Vector3d omega = -axis.cross(axis_rate);
      out(row) = u1.dot(omega);
      out(row + 1) = u2.dot(omega);
    };
  pointing(reference.left_axis, rate.left_axis, 0);
  pointing(reference.right_axis, rate.right_axis, 2);
  out(4) = rate.roll;
  out(5) = rate.distance;
  return out;
}

struct Rows
{
  bool valid = false;        // false when the EE are closer than min_distance
  bool roll_valid = false;   // false when a roll reference is (almost) parallel to d
  double distance = 0.0;
  double roll = 0.0;
  Eigen::Vector3d direction = Eigen::Vector3d::UnitX();
  // Angle between each reference pointing axis and its direction to the other EE.
  double left_angle = 0.0;
  double right_angle = 0.0;
  // reference - measured. The roll row is zero when the roll is undefined.
  Vector6 error = Vector6::Zero();
  // 6 x n; empty when no jacobians were supplied.
  Eigen::MatrixXd jacobian;
};

// Jacobians are 6 x n [linear; angular] in the frame of the poses; pass empty matrices to
// evaluate the error only.
inline Rows evaluate(
  const Eigen::Isometry3d & left, const Eigen::Isometry3d & right,
  const Eigen::MatrixXd & left_jacobian, const Eigen::MatrixXd & right_jacobian,
  const PureAxis & left_goal, const PureAxis & right_goal,
  const Reference & reference, double min_distance)
{
  Rows out;
  const Eigen::Vector3d delta = right.translation() - left.translation();
  const double r = delta.norm();
  out.distance = r;
  if (r < min_distance || r <= 0.0) {return out;}
  out.valid = true;
  const Eigen::Vector3d d = delta / r;
  out.direction = d;
  const Eigen::Matrix3d left_rotation = left.rotation();
  const Eigen::Matrix3d right_rotation = right.rotation();
  const bool jacobians = left_jacobian.size() > 0 && right_jacobian.size() > 0;
  Eigen::MatrixXd left_angular, right_angular, relative_linear, d_jacobian, line_angular;
  if (jacobians) {
    const Eigen::Index cols = left_jacobian.cols();
    left_angular = left_jacobian.middleRows(3, 3);
    right_angular = right_jacobian.middleRows(3, 3);
    relative_linear = right_jacobian.topRows(3) - left_jacobian.topRows(3);
    // d_dot = P (v_right - v_left), P = (I - d d^T) / r.
    d_jacobian = ((Eigen::Matrix3d::Identity() - d * d.transpose()) / r) * relative_linear;
    // Angular velocity of the line direction: omega_d = d x d_dot.
    Eigen::Matrix3d skew;
    skew << 0.0, -d.z(), d.y(), d.z(), 0.0, -d.x(), -d.y(), d.x(), 0.0;
    line_angular = skew * d_jacobian;
    out.jacobian = Eigen::MatrixXd::Zero(6, cols);
  }

  // Pointing rows: rotation vector that takes the EE axis a onto the line direction s,
  // projected on two axes normal to a. Current-rate jacobian: u^T (omega_ee - omega_line).
  const auto pointing_rows = [&](
    const Eigen::Matrix3d & rotation, const Eigen::Vector3d & axis, const Eigen::Vector3d & s,
    const Eigen::MatrixXd & angular, Eigen::Index row, double & angle)
    {
      Eigen::Vector3d local_u1, local_u2;
      pointingBasis(axis, local_u1, local_u2);
      const Eigen::Vector3d a = rotation * axis;
      const Eigen::Vector3d u1 = rotation * local_u1;
      const Eigen::Vector3d u2 = rotation * local_u2;
      const Eigen::Vector3d c = a.cross(s);
      const double sin_angle = c.norm();
      const double cos_angle = a.dot(s);
      angle = std::atan2(sin_angle, cos_angle);
      Eigen::Vector3d rotation_vector = Eigen::Vector3d::Zero();
      if (sin_angle > kAlignedEpsilon) {
        rotation_vector = angle * c / sin_angle;
      } else if (cos_angle < 0.0) {
        rotation_vector = kPi * u1;
      }
      out.error(row) = u1.dot(rotation_vector);
      out.error(row + 1) = u2.dot(rotation_vector);
      if (jacobians) {
        const Eigen::MatrixXd relative_angular = angular - line_angular;
        out.jacobian.row(row) = u1.transpose() * relative_angular;
        out.jacobian.row(row + 1) = u2.transpose() * relative_angular;
      }
    };
  pointing_rows(left_rotation, reference.left_axis, d, left_angular, 0, out.left_angle);
  pointing_rows(right_rotation, reference.right_axis, -d, right_angular, 2, out.right_angle);

  // Roll row: phi = atan2(y, x), y = d.(bL x bR), x = bL.bR - (d.bL)(d.bR).
  const Eigen::Vector3d b_left = rollReferenceAxis(left_rotation, left_goal);
  const Eigen::Vector3d b_right = rollReferenceAxis(right_rotation, right_goal);
  const auto roll = rollGeometry(d, b_left, b_right);
  out.roll_valid = roll.valid;
  out.roll = roll.roll;
  if (roll.valid) {
    out.error(4) = wrapAngle(reference.roll - roll.roll);
    if (jacobians) {
      const double alpha_left = d.dot(b_left);
      const double alpha_right = d.dot(b_right);
      const Eigen::Vector3d bl_x_br = b_left.cross(b_right);
      const Eigen::RowVectorXd dy =
        bl_x_br.transpose() * d_jacobian +
        b_left.cross(b_right.cross(d)).transpose() * left_angular +
        b_right.cross(d.cross(b_left)).transpose() * right_angular;
      const Eigen::RowVectorXd dx =
        -(alpha_right * b_left + alpha_left * b_right).transpose() * d_jacobian +
        (bl_x_br - alpha_right * b_left.cross(d)).transpose() * left_angular +
        (-bl_x_br - alpha_left * b_right.cross(d)).transpose() * right_angular;
      out.jacobian.row(4) = (roll.x * dy - roll.y * dx) / (roll.x * roll.x + roll.y * roll.y);
    }
  } else if (jacobians) {
    // Roll undefined while a reference axis is parallel to the line: hold, keep the main term.
    out.jacobian.row(4) = d.transpose() * (right_angular - left_angular);
  }

  // Distance row.
  out.error(5) = reference.distance - r;
  if (jacobians) {out.jacobian.row(5) = d.transpose() * relative_linear;}
  return out;
}

// Shortest-arc interpolation between unit vectors, with its derivative per unit of u.
inline Eigen::Vector3d interpolateAxis(
  const Eigen::Vector3d & from, const Eigen::Vector3d & to, double u, Eigen::Vector3d * rate = nullptr)
{
  const Eigen::Vector3d c = from.cross(to);
  const double sine = c.norm();
  const double angle = std::atan2(sine, from.dot(to));
  if (sine < kAlignedEpsilon) {
    if (rate) {rate->setZero();}
    return u < 1.0 ? from : to;
  }
  const Eigen::Vector3d normal = c / sine;
  const Eigen::Vector3d out = Eigen::AngleAxisd(u * angle, normal) * from;
  if (rate) {*rate = angle * normal.cross(out);}
  return out;
}

// Linear distance and shortest-arc roll/axes between two references, u in [0, 1].
inline Reference interpolate(
  const Reference & from, const Reference & to, double u, ReferenceRate * rate = nullptr)
{
  u = std::clamp(u, 0.0, 1.0);
  Reference out;
  out.left_axis = interpolateAxis(from.left_axis, to.left_axis, u, rate ? &rate->left_axis : nullptr);
  out.right_axis = interpolateAxis(from.right_axis, to.right_axis, u, rate ? &rate->right_axis : nullptr);
  const double roll_change = wrapAngle(to.roll - from.roll);
  out.roll = wrapAngle(from.roll + u * roll_change);
  out.distance = from.distance + u * (to.distance - from.distance);
  if (rate) {
    rate->roll = roll_change;
    rate->distance = to.distance - from.distance;
  }
  return out;
}

struct Knot
{
  double progress = 0.0;
  Reference reference;
};

// Piecewise path of facing references parametrized by geometric progress in [0, 1]. It
// carries no timing: the caller maps time to progress with the joint trajectory.
class Path
{
public:
  bool init(std::vector<Knot> knots, std::string & error)
  {
    knots_.clear();
    if (knots.size() < 2) {error = "Facing path needs at least two points"; return false;}
    for (std::size_t i = 0; i < knots.size(); ++i) {
      auto & knot = knots[i];
      auto & reference = knot.reference;
      if (!std::isfinite(knot.progress) || knot.progress < 0.0 || knot.progress > 1.0 ||
        (i > 0 && knot.progress <= knots[i - 1].progress))
      {error = "Facing path progress must be strictly increasing in [0, 1]"; return false;}
      if (!reference.left_axis.allFinite() || !reference.right_axis.allFinite() ||
        std::abs(reference.left_axis.norm() - 1.0) > 1e-3 ||
        std::abs(reference.right_axis.norm() - 1.0) > 1e-3)
      {error = "Facing path axes must be unit vectors"; return false;}
      if (!std::isfinite(reference.roll) || !std::isfinite(reference.distance) || reference.distance <= 0.0)
      {error = "Facing path roll must be finite and distance positive"; return false;}
      reference.left_axis.normalize();
      reference.right_axis.normalize();
    }
    if (knots.front().progress != 0.0 || knots.back().progress != 1.0)
    {error = "Facing path must cover progress 0 to 1"; return false;}
    knots_ = std::move(knots);
    return true;
  }

  const std::vector<Knot> & knots() const {return knots_;}

  Reference sample(double progress, ReferenceRate * rate = nullptr) const
  {
    if (knots_.empty()) {return {};}
    progress = std::clamp(progress, 0.0, 1.0);
    std::size_t i = 1;
    while (i + 1 < knots_.size() && knots_[i].progress < progress) {++i;}
    const auto & a = knots_[i - 1];
    const auto & b = knots_[i];
    const double span = b.progress - a.progress;
    const auto out = interpolate(a.reference, b.reference, (progress - a.progress) / span, rate);
    if (rate) {
      rate->left_axis /= span;
      rate->right_axis /= span;
      rate->roll /= span;
      rate->distance /= span;
    }
    return out;
  }

private:
  std::vector<Knot> knots_;
};

// Facing references of a measured configuration and of a goal. The start reference is the
// measured geometry (zero error); its pointing axes are the measured directions to the other
// tool. A NaN roll request keeps the measured roll.
inline bool endpoints(
  const Eigen::Isometry3d & left, const Eigen::Isometry3d & right,
  const PureAxis & left_goal, const PureAxis & right_goal, double roll_request, double distance,
  double min_distance, Reference & start, Reference & target, std::string & error)
{
  Reference goal_axes;
  goal_axes.left_axis = left_goal.vector();
  goal_axes.right_axis = right_goal.vector();
  const auto measured = evaluate(left, right, {}, {}, left_goal, right_goal, goal_axes, min_distance);
  if (!measured.valid) {error = "Initial tools are closer than the facing minimum distance"; return false;}
  if (!measured.roll_valid) {
    error = "Initial facing roll is undefined: a roll reference axis is parallel to the line between the tools";
    return false;
  }
  start.left_axis = (left.rotation().transpose() * measured.direction).normalized();
  start.right_axis = (right.rotation().transpose() * -measured.direction).normalized();
  start.roll = measured.roll;
  start.distance = measured.distance;
  for (const double cosine : {start.left_axis.dot(goal_axes.left_axis),
    start.right_axis.dot(goal_axes.right_axis)})
  {
    if (cosine < -1.0 + 1.0e-6) {
      error = "A tool points exactly away from the other one; the shortest pointing path is ambiguous";
      return false;
    }
  }
  target = goal_axes;
  target.roll = std::isnan(roll_request) ? measured.roll : wrapAngle(roll_request);
  target.distance = distance;
  return true;
}

// Largest change between two references: distance [m] and angle [rad] (roll or pointing).
inline void changes(const Reference & from, const Reference & to, double & distance, double & angle)
{
  const auto between = [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
      return std::atan2(a.cross(b).norm(), a.dot(b));
    };
  distance = std::abs(to.distance - from.distance);
  angle = std::max({std::abs(wrapAngle(to.roll - from.roll)), between(from.left_axis, to.left_axis),
    between(from.right_axis, to.right_axis)});
}

// Facing path from `from` to `to`. With 0 < fraction < 1 and both an angular and a distance
// change it has two stages: up to s = fraction the pointing axes and the roll reach the goal
// at the start distance; then only the distance changes. Each stage is eased (zero rate at its
// ends) through intermediate knots, so linear interpolation between knots keeps it, and there
// is an exact knot at s = fraction. A positive stage_distance is reached during the first
// stage instead of keeping the start distance, and always creates both stages. Changes below min_angle [rad] or min_distance [m] do not
// get a stage of their own. `used` receives the stage boundary, or 0 when simultaneous.
inline std::vector<Knot> stagedKnots(
  const Reference & from, const Reference & to, double fraction, double * used = nullptr,
  double min_angle = 0.05, double min_distance = 0.002, int knots_per_stage = 8,
  double stage_distance = -1.0)
{
  double distance = 0.0, angle = 0.0;
  changes(from, to, distance, angle);
  // A requested distance at the boundary always creates both stages.
  const bool requested = stage_distance > 0.0;
  const bool staged = fraction > 0.0 && fraction < 1.0 &&
    (requested || (distance >= min_distance && angle >= min_angle));
  if (used) {*used = staged ? fraction : 0.0;}
  if (!staged) {return {{0.0, from}, {1.0, to}};}
  Reference oriented = to;
  oriented.distance = requested ? stage_distance : from.distance;
  std::vector<Knot> out{{0.0, from}};
  const auto stage = [&](const Reference & a, const Reference & b, double s0, double s1) {
      for (int i = 1; i <= knots_per_stage; ++i) {
        const double u = static_cast<double>(i) / knots_per_stage;
        const double eased = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
        out.push_back({i == knots_per_stage ? s1 : s0 + (s1 - s0) * u,
          i == knots_per_stage ? b : interpolate(a, b, eased)});
      }
    };
  stage(from, oriented, 0.0, fraction);
  stage(oriented, to, fraction, 1.0);
  return out;
}

// Rest-to-rest quintic progress over a duration, as the Cartesian pose profile.
inline double restToRestProgress(double time, double duration, double * rate = nullptr)
{
  if (rate) {*rate = 0.0;}
  if (!(duration > 0.0)) {return 1.0;}
  const double u = std::clamp(time / duration, 0.0, 1.0);
  if (rate && u > 0.0 && u < 1.0) {*rate = 30.0 * u * u * (1.0 - u) * (1.0 - u) / duration;}
  return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
}

// Duration of the rest-to-rest profile that keeps the peak rates within the limits.
inline double restToRestDuration(
  const Reference & from, const Reference & to, double linear_velocity, double angular_velocity,
  double minimum = 1.0)
{
  double distance = 0.0, angle = 0.0;
  changes(from, to, distance, angle);
  return std::max(minimum, 1.875 * std::max(distance / linear_velocity, angle / angular_velocity));
}

// Progress as a function of the joint trajectory time: piecewise linear between the
// (time, progress) pairs of its points. rate receives ds/dt.
inline double progressAtTime(
  const std::vector<double> & times, const std::vector<double> & progress, double time,
  double * rate = nullptr)
{
  if (rate) {*rate = 0.0;}
  if (times.empty() || times.size() != progress.size()) {return 0.0;}
  if (time <= times.front()) {return progress.front();}
  if (time >= times.back()) {return progress.back();}
  std::size_t i = 1;
  while (i + 1 < times.size() && times[i] < time) {++i;}
  const double span = times[i] - times[i - 1];
  if (span <= 0.0) {return progress[i];}
  const double slope = (progress[i] - progress[i - 1]) / span;
  if (rate) {*rate = slope;}
  return progress[i - 1] + slope * (time - times[i - 1]);
}

}  // namespace sura_manipulation_actions::facing
