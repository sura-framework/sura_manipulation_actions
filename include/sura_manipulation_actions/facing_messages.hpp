#pragma once

#include "sura_manipulation_actions/facing_geometry.hpp"
#include "sura_manipulation_actions/msg/facing_path.hpp"
#include "sura_manipulation_actions/msg/facing_reference.hpp"

namespace sura_manipulation_actions::facing
{

inline Eigen::Vector3d fromMessage(const geometry_msgs::msg::Vector3 & message)
{
  return {message.x, message.y, message.z};
}

inline geometry_msgs::msg::Vector3 toMessage(const Eigen::Vector3d & value)
{
  geometry_msgs::msg::Vector3 message;
  message.x = value.x(); message.y = value.y(); message.z = value.z();
  return message;
}

inline msg::FacingPath toMessage(
  const PureAxis & left_goal, const PureAxis & right_goal, const std::vector<Knot> & knots)
{
  msg::FacingPath message;
  message.left_goal_axis = toMessage(left_goal.vector());
  message.right_goal_axis = toMessage(right_goal.vector());
  for (const auto & knot : knots) {
    msg::FacingPathPoint point;
    point.progress = knot.progress;
    point.left_pointing_axis = toMessage(knot.reference.left_axis);
    point.right_pointing_axis = toMessage(knot.reference.right_axis);
    point.roll_angle = knot.reference.roll;
    point.distance = knot.reference.distance;
    message.points.push_back(point);
  }
  return message;
}

inline bool fromMessage(
  const msg::FacingPath & message, PureAxis & left_goal, PureAxis & right_goal, Path & path,
  std::string & error)
{
  if (!pureAxis(fromMessage(message.left_goal_axis), left_goal, error) ||
    !pureAxis(fromMessage(message.right_goal_axis), right_goal, error)) {return false;}
  std::vector<Knot> knots;
  for (const auto & point : message.points) {
    Knot knot;
    knot.progress = point.progress;
    knot.reference.left_axis = fromMessage(point.left_pointing_axis);
    knot.reference.right_axis = fromMessage(point.right_pointing_axis);
    knot.reference.roll = point.roll_angle;
    knot.reference.distance = point.distance;
    knots.push_back(knot);
  }
  return path.init(std::move(knots), error);
}

// Reference at a progress value; progress_rate (ds/dt) turns path derivatives into feedforward.
inline msg::FacingReference referenceMessage(
  const PureAxis & left_goal, const PureAxis & right_goal, const Path & path,
  double progress, double progress_rate)
{
  ReferenceRate rate;
  const auto reference = path.sample(progress, &rate);
  msg::FacingReference message;
  message.left_goal_axis = toMessage(left_goal.vector());
  message.right_goal_axis = toMessage(right_goal.vector());
  message.left_pointing_axis = toMessage(reference.left_axis);
  message.right_pointing_axis = toMessage(reference.right_axis);
  message.roll_angle = reference.roll;
  message.distance = reference.distance;
  message.left_pointing_rate = toMessage(rate.left_axis * progress_rate);
  message.right_pointing_rate = toMessage(rate.right_axis * progress_rate);
  message.roll_rate = rate.roll * progress_rate;
  message.distance_rate = rate.distance * progress_rate;
  return message;
}

}  // namespace sura_manipulation_actions::facing
