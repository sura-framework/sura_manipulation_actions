#pragma once

#include <Eigen/Geometry>

namespace sura_manipulation_actions
{
// Task activation selects world-frame axes. Project there before expressing
// the error in the (possibly rotated) tolerance frame.
inline Eigen::Matrix<double, 6, 1> cartesianTrackingError(
  const Eigen::Isometry3d & desired, const Eigen::Isometry3d & measured,
  const Eigen::Matrix3d & tolerance_axes,
  const Eigen::Matrix<double, 6, 1> & activation = Eigen::Matrix<double, 6, 1>::Ones())
{
  Eigen::Matrix<double, 6, 1> result;
  const Eigen::Vector3d position = desired.translation() - measured.translation();
  const Eigen::AngleAxisd rotation(desired.rotation() * measured.rotation().transpose());
  const Eigen::Vector3d angular = rotation.axis() * rotation.angle();
  result.head<3>() = tolerance_axes.transpose() * position.cwiseProduct(activation.head<3>());
  result.tail<3>() = tolerance_axes.transpose() * angular.cwiseProduct(activation.tail<3>());
  return result;
}
}  // namespace sura_manipulation_actions
