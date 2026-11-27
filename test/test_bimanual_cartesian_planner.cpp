#include "sura_manipulation_actions/bimanual_cartesian_planner.hpp"
#include "sura_manipulation_actions/cartesian_trajectory_timing.hpp"
#include "sura_manipulation_actions/single_arm_cartesian_planner.hpp"
#include "sura_manipulation_actions/base_trajectory_sampling.hpp"

#include <gtest/gtest.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace sura_manipulation_actions::bimanual_cartesian
{
namespace
{

struct SyntheticRobot
{
  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScenePtr scene;
  Arm left, right;
  Pair pair;
  std::string spare_joint = "spare_slide";

  explicit SyntheticRobot(
    int left_dofs = 4, int right_dofs = 6, double left_yaw = 0.0, double right_yaw = 0.0,
    bool floating = false)
  {
    std::ostringstream urdf;
    urdf << "<robot name='synthetic'><link name='world'/>"
         << "<link name='reference'/><joint name='reference_joint' type='fixed'>"
         << "<parent link='world'/><child link='reference'/>"
         << "<origin xyz='1.2 -0.7 0.3' rpy='0.4 -0.3 0.5'/></joint>";
    auto add_arm = [&](const std::string & name, int count, double x, double yaw) {
        Arm arm;
        arm.name = name;
        arm.group_name = name;
        arm.tip_frame = name + "_tip";
        urdf << "<link name='" << name << "_mount'/><joint name='" << name
             << "_mount_joint' type='fixed'><parent link='world'/><child link='" << name
             << "_mount'/><origin xyz='" << x << " 0 0' rpy='0 0 " << yaw << "'/></joint>";
        std::string parent = name + "_mount";
        for (int i = 0; i < count; ++i) {
          const std::string child = i + 1 == count ? arm.tip_frame : name + "_link_" + std::to_string(i);
          const std::string joint = name + "_joint_" + std::to_string(i);
          arm.joint_names.push_back(joint);
          urdf << "<link name='" << child << "'>";
          if (i + 1 == count) {
            urdf << "<collision><geometry><sphere radius='0.004'/></geometry></collision>";
          }
          urdf << "</link><joint name='" << joint << "' type='"
               << (i < 3 ? "prismatic" : "revolute") << "'><parent link='" << parent
               << "'/><child link='" << child << "'/><axis xyz='";
          if (i == 0 || i == 5) {urdf << "1 0 0";}
          else if (i == 1 || i == 4) {urdf << "0 1 0";}
          else {urdf << "0 0 1";}
          urdf << "'/><limit lower='-1' upper='1' effort='1' velocity='1'/></joint>";
          parent = child;
        }
        return arm;
      };
    left = add_arm("left", left_dofs, -0.3, left_yaw);
    right = add_arm("right", right_dofs, 0.3, right_yaw);
    // A non-commanded robot joint allows execution revalidation to exercise
    // current full-body collision state, independently of the planned arms.
    urdf << "<link name='spare_tool'><collision><geometry><sphere radius='0.004'/>"
         << "</geometry></collision></link><joint name='spare_slide' type='prismatic'>"
         << "<parent link='world'/><child link='spare_tool'/><origin xyz='2 0 0'/>"
         << "<axis xyz='1 0 0'/><limit lower='-2' upper='2' effort='1' velocity='1'/>"
         << "</joint></robot>";
    const auto urdf_model = urdf::parseURDF(urdf.str());
    if (!urdf_model) {throw std::runtime_error("Synthetic URDF could not be parsed");}
    auto semantic = std::make_shared<srdf::Model>();
    std::string srdf =
      "<robot name='synthetic'><group name='left'><chain base_link='left_mount' tip_link='left_tip'/></group>"
      "<group name='right'><chain base_link='right_mount' tip_link='right_tip'/></group>"
      "<group name='both'><group name='left'/><group name='right'/></group>";
    if (floating) {
      srdf += "<virtual_joint name='world_joint' type='floating' parent_frame='world_ned' child_link='world'/>"
        "<group name='base_left'><joint name='world_joint'/><group name='left'/></group>"
        "<group name='base_right'><joint name='world_joint'/><group name='right'/></group>";
      left.whole_body_group_name = "base_left";
      right.whole_body_group_name = "base_right";
    }
    srdf += "</robot>";
    if (!semantic->initString(*urdf_model, srdf)) {throw std::runtime_error("Synthetic SRDF could not be parsed");}
    model = std::make_shared<moveit::core::RobotModel>(urdf_model, semantic);
    for (auto * joint : model->getActiveJointModels()) {
      if (joint->getVariableCount() != 1) {continue;}
      const auto & name = joint->getVariableNames().front();
      auto bounds = joint->getVariableBounds(name);
      bounds.velocity_bounded_ = true;
      bounds.min_velocity_ = -1.0;
      bounds.max_velocity_ = 1.0;
      bounds.acceleration_bounded_ = true;
      bounds.min_acceleration_ = -2.0;
      bounds.max_acceleration_ = 2.0;
      joint->setVariableBounds(name, bounds);
    }
    scene = std::make_shared<planning_scene::PlanningScene>(model);
    scene->getCurrentStateNonConst().setToDefaultValues();
    for (const auto & name : model->getVariableNames()) {
      scene->getCurrentStateNonConst().setVariablePosition(name, 0.0);
    }
    if (floating) {
      scene->getCurrentStateNonConst().setJointPositions("world_joint", Eigen::Isometry3d::Identity());
    }
    scene->getCurrentStateNonConst().update();
    pair.arm_names = {left.name, right.name};
    pair.group_name = "both";
  }

  // Facing goal that changes the measured tool distance by x. The mounts sit on the
  // world X axis, so +x of the left tool and -x of the right tool face each other.
  GoalSpec goal(double x = 0.02, bool swap = false) const
  {
    const auto & state = scene->getCurrentState();
    const auto & arm_a = swap ? right : left;
    const auto & arm_b = swap ? left : right;
    GoalSpec goal;
    goal.reference_frame = model->getModelFrame();
    goal.facing.left_pointing_axis = (swap ? -1.0 : 1.0) * Eigen::Vector3d::UnitX();
    goal.facing.right_pointing_axis = (swap ? 1.0 : -1.0) * Eigen::Vector3d::UnitX();
    goal.facing.distance = (state.getGlobalLinkTransform(arm_b.tip_frame).translation() -
      state.getGlobalLinkTransform(arm_a.tip_frame).translation()).norm() + x;
    goal.facing.pointing_tolerance = 0.005;
    goal.facing.roll_tolerance = 0.005;
    goal.facing.distance_tolerance = 0.002;
    goal.planning_time = 5.0;
    return goal;
  }

  static Options options()
  {
    Options options;
    options.optimize_trajectory = false;
    options.velocity_scaling = 0.5;
    options.acceleration_scaling = 0.5;
    options.recovery_seeds = 2;
    options.max_subdivision_depth = 3;
    return options;
  }

  void obstacle(double x)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = model->getModelFrame();
    object.id = "new_obstacle";
    object.operation = object.ADD;
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {0.004, 0.02, 0.02};
    object.primitives.push_back(box);
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.orientation.w = 1.0;
    object.primitive_poses.push_back(pose);
    ASSERT_TRUE(scene->processCollisionObjectMsg(object));
  }
};

trajectory_msgs::msg::JointTrajectory singleJointTrajectory()
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.joint_names = {"joint"};
  trajectory_msgs::msg::JointTrajectoryPoint a, b;
  a.positions = {0.0}; a.velocities = {0.0}; a.accelerations = {0.0};
  b.positions = {1.0}; b.velocities = {0.0}; b.accelerations = {0.0};
  b.time_from_start.sec = 2;
  msg.points = {a, b};
  return msg;
}

void restBetweenOriginalEndpoints(PlannedTrajectory & plan)
{
  // Planning may insert intermediate poses while repairing IK or timing. This
  // regression specifically exercises one rest-to-rest quintic between the
  // unchanged planned endpoints, independently of those adaptive subdivisions.
  const auto first = plan.joint_trajectory.points.front();
  const auto last = plan.joint_trajectory.points.back();
  plan.joint_trajectory.points = {first, last};
  plan.progress_values = {0.0, 1.0};
  for (std::size_t i = 0; i < plan.joint_trajectory.points.size(); ++i) {
    auto & point = plan.joint_trajectory.points[i];
    point.velocities.assign(point.positions.size(), 0.0);
    point.accelerations.assign(point.positions.size(), 0.0);
    point.time_from_start = timing::duration(2.0 * static_cast<double>(i));
  }
  auto endpoints = std::make_shared<robot_trajectory::RobotTrajectory>(
    plan.trajectory->getRobotModel(), plan.trajectory->getGroupName());
  endpoints->setRobotTrajectoryMsg(plan.trajectory->getFirstWayPoint(), plan.joint_trajectory);
  plan.trajectory = std::move(endpoints);
}

TEST(BimanualCartesianInterpolation, QuinticBoundaryDerivativesAreActualDerivatives)
{
  const auto msg = singleJointTrajectory();
  std::string error;
  trajectory_msgs::msg::JointTrajectoryPoint sample;
  ASSERT_TRUE(BimanualCartesianPlanner::sampleTrajectory(msg, 1.0, sample, error)) << error;
  EXPECT_NEAR(sample.positions[0], 0.5, 1.0e-12);
  EXPECT_NEAR(sample.velocities[0], 0.9375, 1.0e-12);
  EXPECT_NEAR(sample.accelerations[0], 0.0, 1.0e-12);
  ASSERT_TRUE(BimanualCartesianPlanner::sampleTrajectory(msg, 0.5, sample, error));
  EXPECT_NEAR(sample.positions[0], 0.103515625, 1.0e-12);
  EXPECT_NEAR(sample.accelerations[0], 1.40625, 1.0e-12);
}

TEST(BimanualCartesianInterpolation, PositionOnlyAndCubicInputsUseTheirBoundaryPolynomials)
{
  auto msg = singleJointTrajectory();
  for (auto & point : msg.points) {point.accelerations.clear();}
  std::string error;
  trajectory_msgs::msg::JointTrajectoryPoint sample;
  ASSERT_TRUE(BimanualCartesianPlanner::sampleTrajectory(msg, 0.5, sample, error));
  EXPECT_NEAR(sample.positions[0], 0.15625, 1.0e-12);
  for (auto & point : msg.points) {point.velocities.clear();}
  ASSERT_TRUE(BimanualCartesianPlanner::sampleTrajectory(msg, 0.5, sample, error));
  EXPECT_NEAR(sample.positions[0], 0.25, 1.0e-12);
  EXPECT_NEAR(sample.velocities[0], 0.5, 1.0e-12);
}

TEST(BimanualCartesianInterpolation, RejectsMalformedTimesAndDerivativeArrays)
{
  auto msg = singleJointTrajectory();
  std::string error;
  trajectory_msgs::msg::JointTrajectoryPoint sample;
  msg.points[1].time_from_start.sec = 0;
  EXPECT_FALSE(BimanualCartesianPlanner::sampleTrajectory(msg, 0.5, sample, error));
  msg = singleJointTrajectory();
  msg.points[1].accelerations.push_back(0.0);
  EXPECT_FALSE(BimanualCartesianPlanner::sampleTrajectory(msg, 0.5, sample, error));
}

TEST(BimanualCartesianPlanner, GenericDifferentDofArmsProduceOneTimedTrajectory)
{
  SyntheticRobot robot(4, 6);
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  const auto goal = robot.goal();
  const auto options = robot.options();
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  ASSERT_TRUE(plan.trajectory);
  ASSERT_EQ(plan.joint_trajectory.joint_names.size(), 10U);
  ASSERT_GE(plan.joint_trajectory.points.size(), 2U);
  EXPECT_EQ(plan.progress_values.size(), plan.joint_trajectory.points.size());
  double previous = -1.0;
  bool has_motion_derivative = false;
  for (const auto & point : plan.joint_trajectory.points) {
    EXPECT_EQ(point.positions.size(), 10U);
    EXPECT_EQ(point.velocities.size(), 10U);
    EXPECT_EQ(point.accelerations.size(), 10U);
    const double time = point.time_from_start.sec + point.time_from_start.nanosec * 1.0e-9;
    EXPECT_GT(time, previous);
    previous = time;
    for (const auto velocity : point.velocities) {has_motion_derivative |= std::abs(velocity) > 1.0e-8;}
  }
  EXPECT_TRUE(has_motion_derivative);
  ASSERT_TRUE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error)) << error;
}

TEST(BimanualCartesianPlanner, EqualWeightsDistributeMotionAcrossBothArms)
{
  SyntheticRobot robot(4, 4);
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  auto goal = robot.goal();
  goal.facing.distance_tolerance = 0.0;
  auto options = robot.options();
  options.weights.assign(8, 1.0);
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  const auto & final = plan.trajectory->getLastWayPoint();
  const double left = final.getVariablePosition(robot.left.joint_names.front());
  const double right = final.getVariablePosition(robot.right.joint_names.front());
  EXPECT_LT(left, -0.005);
  EXPECT_GT(right, 0.005);
  EXPECT_NEAR(left, -right, options.position_epsilon);
  EXPECT_NEAR(right - left, 0.02, options.position_epsilon);
  EXPECT_EQ(plan.joint_trajectory.points.front().positions, (std::vector<double>(8, 0.0)));
  ASSERT_TRUE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error)) << error;
}

TEST(BimanualCartesianPlanner, LargerWeightReducesMotionFromMeasuredInitialPosition)
{
  SyntheticRobot robot(4, 4);
  auto & start = robot.scene->getCurrentStateNonConst();
  const auto & left_joint = robot.left.joint_names.front();
  const auto & right_joint = robot.right.joint_names.front();
  start.setVariablePosition(left_joint, 0.05);
  start.setVariablePosition(right_joint, -0.03);
  start.update();
  auto goal = robot.goal();
  goal.facing.distance_tolerance = 0.0;
  auto options = robot.options();
  options.weights.assign(8, 0.0);
  const auto & names = robot.model->getJointModelGroup(robot.pair.group_name)->getVariableNames();
  const auto index = std::distance(names.begin(), std::find(names.begin(), names.end(), left_joint));
  ASSERT_LT(static_cast<std::size_t>(index), options.weights.size());
  BimanualCartesianPlanner planner;
  PlannedTrajectory unweighted, weighted;
  std::string error;
  ASSERT_TRUE(planner.plan(robot.scene, start, robot.left, robot.right, robot.pair,
    goal, options, unweighted, error)) << error;
  options.weights[static_cast<std::size_t>(index)] = 100.0;
  ASSERT_TRUE(planner.plan(robot.scene, start, robot.left, robot.right, robot.pair,
    goal, options, weighted, error)) << error;
  const auto & baseline = unweighted.trajectory->getLastWayPoint();
  const auto & final = weighted.trajectory->getLastWayPoint();
  EXPECT_LT(std::abs(final.getVariablePosition(left_joint) - 0.05),
    0.5 * std::abs(baseline.getVariablePosition(left_joint) - 0.05));
  EXPECT_GT(final.getVariablePosition(right_joint) + 0.03,
    baseline.getVariablePosition(right_joint) + 0.03);
}

TEST(BimanualCartesianPlanner, OmittedAndZeroWeightsLeaveBothArmsFree)
{
  SyntheticRobot robot(4, 4);
  BimanualCartesianPlanner planner;
  PlannedTrajectory omitted, zero;
  std::string error;
  auto options = robot.options();
  const auto goal = robot.goal();
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, omitted, error)) << error;
  options.weights.assign(8, 0.0);
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, zero, error)) << error;
  const auto & final = zero.trajectory->getLastWayPoint();
  EXPECT_LT(final.getVariablePosition(robot.left.joint_names.front()), -0.005);
  EXPECT_GT(final.getVariablePosition(robot.right.joint_names.front()), 0.005);
  for (const auto & name : zero.joint_trajectory.joint_names) {
    EXPECT_NEAR(final.getVariablePosition(name),
      omitted.trajectory->getLastWayPoint().getVariablePosition(name), 1e-9);
  }
}

TEST(BimanualCartesianPlanner, RejectsInvalidWeightsInPlanningAndRevalidation)
{
  SyntheticRobot robot(4, 4);
  BimanualCartesianPlanner planner;
  PlannedTrajectory valid, invalid;
  std::string error;
  auto options = robot.options();
  const auto goal = robot.goal();
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, valid, error)) << error;
  const std::vector<std::vector<double>> invalid_weights{
    std::vector<double>(7, 0.0), std::vector<double>(9, 0.0),
    std::vector<double>(8, -0.1),
    std::vector<double>(8, std::numeric_limits<double>::quiet_NaN()),
    std::vector<double>(8, std::numeric_limits<double>::infinity())};
  for (const auto & weights : invalid_weights) {
    options.weights = weights;
    EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
      robot.right, robot.pair, goal, options, invalid, error));
    EXPECT_FALSE(invalid.trajectory);
    EXPECT_NE(error.find("planning.weights"), std::string::npos);
    EXPECT_FALSE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
      goal, options, valid, error));
    EXPECT_NE(error.find("planning.weights"), std::string::npos);
  }
}

TEST(BimanualFacingPlanner, ReferenceStartsAtMeasuredGeometryAndReachesTheGoal)
{
  SyntheticRobot robot(6, 6);
  auto & start = robot.scene->getCurrentStateNonConst();
  // Both tools start misaligned: yaw on the left, pitch on the right.
  start.setVariablePosition(robot.left.joint_names[3], 0.15);
  start.setVariablePosition(robot.right.joint_names[4], -0.1);
  start.update();
  const auto goal = robot.goal(-0.03);
  const auto options = robot.options();
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  ASSERT_TRUE(planner.plan(robot.scene, start, robot.left, robot.right, robot.pair,
    goal, options, plan, error)) << error;
  const auto rows = [&](const moveit::core::RobotState & state, const facing::Reference & reference) {
      return facing::evaluate(state.getGlobalLinkTransform(robot.left.tip_frame),
        state.getGlobalLinkTransform(robot.right.tip_frame), {}, {}, plan.facing_left_goal,
        plan.facing_right_goal, reference, options.facing_min_distance);
    };
  facing::Path path;
  ASSERT_TRUE(path.init(plan.facing_path, error)) << error;
  EXPECT_LT(rows(start, path.sample(0.0)).error.cwiseAbs().maxCoeff(), 1.0e-9);
  EXPECT_DOUBLE_EQ(plan.facing_path.front().progress, 0.0);
  EXPECT_DOUBLE_EQ(plan.facing_path.back().progress, 1.0);
  const auto final = rows(plan.trajectory->getLastWayPoint(), plan.facing_path.back().reference);
  EXPECT_LE(final.left_angle, goal.facing.pointing_tolerance + 1.0e-6);
  EXPECT_LE(final.right_angle, goal.facing.pointing_tolerance + 1.0e-6);
  EXPECT_NEAR(final.distance, goal.facing.distance, goal.facing.distance_tolerance + 1.0e-6);
  EXPECT_LT((plan.facing_path.back().reference.left_axis - Eigen::Vector3d::UnitX()).norm(), 1.0e-9);
  EXPECT_LT((plan.facing_path.back().reference.right_axis + Eigen::Vector3d::UnitX()).norm(), 1.0e-9);
  facing::Path expected_path;
  ASSERT_TRUE(expected_path.init(facing::stagedKnots(plan.facing_start, plan.facing_target,
    options.facing_orientation_fraction), error)) << error;
  for (const double s : {0.0, 0.13, 0.4, 0.5, 0.87, 1.0}) {
    const auto expected = expected_path.sample(s);
    const auto sampled = path.sample(s);
    EXPECT_LT((sampled.left_axis - expected.left_axis).norm(), 1.0e-9);
    EXPECT_NEAR(sampled.distance, expected.distance, 1.0e-9);
  }
  // Pointing first at the start distance, then the distance.
  for (std::size_t i = 0; i < plan.progress_values.size(); ++i) {
    const auto & state = plan.trajectory->getWayPoint(i);
    const auto measured = rows(state, path.sample(plan.progress_values[i]));
    if (plan.progress_values[i] <= options.facing_orientation_fraction) {
      EXPECT_NEAR(measured.distance, plan.facing_start.distance, goal.facing.distance_tolerance + 1e-6);
    } else {
      EXPECT_LE(measured.left_angle, goal.facing.pointing_tolerance + 1e-6);
    }
  }
  ASSERT_TRUE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error)) << error;
}

TEST(BimanualFacingPlanner, JointSpaceOrientationPhaseReachesItsDistanceThenFollowsTheFinalOne)
{
  SyntheticRobot robot(6, 6);
  auto start = robot.scene->getCurrentState();
  start.setVariablePosition(robot.left.joint_names[3], 0.15);
  start.setVariablePosition(robot.right.joint_names[4], -0.1);
  start.update();
  const double initial = (start.getGlobalLinkTransform(robot.right.tip_frame).translation() -
    start.getGlobalLinkTransform(robot.left.tip_frame).translation()).norm();
  auto goal = robot.goal(0.0);
  goal.facing.orientation_phase_distance = initial + 0.04;
  goal.facing.distance = initial - 0.03;
  goal.planning_time = 20.0;
  auto options = robot.options();
  options.facing_joint_phase = true;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  ASSERT_TRUE(planner.plan(robot.scene, start, robot.left, robot.right, robot.pair,
    goal, options, plan, error)) << error;
  facing::Path path;
  ASSERT_TRUE(path.init(plan.facing_path, error)) << error;
  EXPECT_NEAR(path.sample(0.4).distance, initial + 0.04, 1e-6);
  EXPECT_NEAR(path.sample(1.0).distance, initial - 0.03, 1e-6);
  EXPECT_NE(std::find(plan.progress_values.begin(), plan.progress_values.end(), 0.4), plan.progress_values.end());
  // The derived reference follows the motion in both phases, within the goal tolerances.
  for (std::size_t i = 0; i < plan.progress_values.size(); ++i) {
    const auto & state = plan.trajectory->getWayPoint(i);
    const auto rows = facing::evaluate(state.getGlobalLinkTransform(robot.left.tip_frame),
      state.getGlobalLinkTransform(robot.right.tip_frame), {}, {}, plan.facing_left_goal,
      plan.facing_right_goal, path.sample(plan.progress_values[i]), options.facing_min_distance);
    EXPECT_LE(rows.left_angle, goal.facing.pointing_tolerance + 2e-3) << i;
    EXPECT_LE(rows.right_angle, goal.facing.pointing_tolerance + 2e-3) << i;
    EXPECT_LE(std::abs(rows.error(5)), goal.facing.distance_tolerance + 1e-3) << i;
    const double distance = path.sample(plan.progress_values[i]).distance - rows.error(5);
    if (plan.progress_values[i] < 0.4) {
      EXPECT_GE(distance, initial - options.facing_phase_distance_margin - 1e-6) << i;
      EXPECT_LE(distance, initial + 0.04 + options.facing_phase_distance_margin + 1e-6) << i;
    }
  }
  ASSERT_TRUE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error)) << error;
  // A maximum tool distance below the requested phase distance is rejected.
  options.facing_phase_max_distance = initial + 0.01;
  EXPECT_FALSE(planner.plan(robot.scene, start, robot.left, robot.right, robot.pair,
    goal, options, plan, error));
  EXPECT_NE(error.find("facing_phase_max_distance"), std::string::npos) << error;
}

TEST(BimanualFacingGeometry, StagedKnotsReachTheRequestedStageDistance)
{
  facing::Reference from, to;
  from.distance = 0.4;
  to = from;
  to.distance = 0.3;
  double used = -1.0;
  facing::Path path;
  std::string error;
  ASSERT_TRUE(path.init(facing::stagedKnots(from, to, 0.4, &used, 0.05, 0.002, 8, 0.5), error)) << error;
  EXPECT_DOUBLE_EQ(used, 0.4);
  EXPECT_NEAR(path.sample(0.4).distance, 0.5, 1e-12);
  EXPECT_NEAR(path.sample(1.0).distance, 0.3, 1e-12);
}

TEST(BimanualFacingGeometry, StagedKnotsOrientBeforeChangingTheDistance)
{
  facing::Reference from, to;
  from.left_axis = Eigen::Vector3d(1.0, 0.3, 0.0).normalized();
  from.right_axis = -Eigen::Vector3d::UnitX();
  from.roll = 0.2;
  from.distance = 0.4;
  to.left_axis = Eigen::Vector3d::UnitX();
  to.right_axis = -Eigen::Vector3d::UnitX();
  to.roll = -0.1;
  to.distance = 0.3;
  double used = -1.0;
  const auto knots = facing::stagedKnots(from, to, 0.4, &used);
  EXPECT_DOUBLE_EQ(used, 0.4);
  facing::Path path;
  std::string error;
  ASSERT_TRUE(path.init(knots, error)) << error;
  const auto middle = path.sample(0.4);
  EXPECT_NEAR(middle.distance, 0.4, 1e-12);
  EXPECT_NEAR(middle.roll, -0.1, 1e-12);
  EXPECT_LT((middle.left_axis - to.left_axis).norm(), 1e-12);
  EXPECT_NEAR(path.sample(0.2).distance, 0.4, 1e-12);
  EXPECT_NEAR(path.sample(1.0).distance, 0.3, 1e-12);
  EXPECT_NEAR(path.sample(0.7).roll, -0.1, 1e-12);
  // Eased stages: almost no change in the first part of each stage.
  EXPECT_LT(std::abs(path.sample(0.4 + 0.6 / 8).distance - 0.4), 0.01);
  // Zero fraction, or a single kind of change, keeps the simultaneous path.
  EXPECT_EQ(facing::stagedKnots(from, to, 0.0, &used).size(), 2u);
  EXPECT_DOUBLE_EQ(used, 0.0);
  auto distance_only = from;
  distance_only.distance = 0.3;
  EXPECT_EQ(facing::stagedKnots(from, distance_only, 0.4).size(), 2u);
  // An angular change below the threshold does not get its own stage.
  auto almost = distance_only;
  almost.roll = from.roll + 0.02;
  EXPECT_EQ(facing::stagedKnots(from, almost, 0.4, &used, 0.05).size(), 2u);
}

TEST(BimanualFacingPlanner, ReachesRequestedRollAndKeepsMeasuredRollForNan)
{
  SyntheticRobot robot(6, 6);
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  auto goal = robot.goal(0.0);
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, robot.options(), plan, error)) << error;
  EXPECT_NEAR(plan.facing_target.roll, plan.facing_start.roll, 1.0e-12);
  goal.facing.roll = facing::wrapAngle(plan.facing_start.roll + 0.1);
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, robot.options(), plan, error)) << error;
  const auto & last = plan.trajectory->getLastWayPoint();
  const auto rows = facing::evaluate(last.getGlobalLinkTransform(robot.left.tip_frame),
    last.getGlobalLinkTransform(robot.right.tip_frame), {}, {}, plan.facing_left_goal,
    plan.facing_right_goal, plan.facing_target, 0.02);
  EXPECT_LE(std::abs(rows.error(4)), goal.facing.roll_tolerance + 1.0e-6);
}

TEST(BimanualFacingPlanner, RejectsUndefinedRollShortDistanceAndAmbiguousPointing)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  const auto fails = [&](const GoalSpec & goal, const std::string & text) {
      EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
        robot.right, robot.pair, goal, robot.options(), plan, error));
      EXPECT_NE(error.find(text), std::string::npos) << error;
      EXPECT_FALSE(plan.trajectory);
    };
  auto goal = robot.goal();
  goal.facing.left_pointing_axis = Eigen::Vector3d::UnitZ();  // roll reference +x is along d
  fails(goal, "roll is undefined");
  goal = robot.goal();
  goal.facing.distance = 0.01;
  fails(goal, "minimum distance");
  goal = robot.goal();
  goal.facing.right_pointing_axis = Eigen::Vector3d::UnitX();
  fails(goal, "ambiguous");
  goal = robot.goal();
  goal.facing.left_pointing_axis = Eigen::Vector3d(1.0, 1.0, 0.0);
  fails(goal, "pure signed axis");
}

TEST(BimanualFacingPlanner, FloatingBaseStaysFixed)
{
  SyntheticRobot robot(4, 4, 0.0, 0.0, true);
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, robot.goal(), robot.options(), plan, error)) << error;
  EXPECT_EQ(plan.joint_trajectory.joint_names.size(), 8U);
  EXPECT_TRUE(plan.base_trajectory.points.empty());
  for (std::size_t i = 0; i < plan.trajectory->getWayPointCount(); ++i) {
    EXPECT_LT((plan.trajectory->getWayPoint(i).getGlobalLinkTransform("world").matrix() -
      Eigen::Matrix4d::Identity()).norm(), 1.0e-12);
  }
}

// Two free bodies: columns are [v_left, w_left, v_right, w_right] in world axes.
TEST(BimanualFacingGeometry, JacobianMatchesFiniteDifferences)
{
  Eigen::Isometry3d left = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d right = Eigen::Isometry3d::Identity();
  left.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.2, 0.5, 1.0).normalized()).toRotationMatrix();
  right.translation() = Eigen::Vector3d(0.5, 0.1, -0.2);
  right.linear() = Eigen::AngleAxisd(-0.9, Eigen::Vector3d(1.0, 0.3, 0.4).normalized()).toRotationMatrix();
  facing::PureAxis left_goal{0, 1.0}, right_goal{0, -1.0};
  const Eigen::Vector3d d = (right.translation() - left.translation()).normalized();
  // References aligned with the current geometry, so the pointing error is zero.
  facing::Reference reference;
  reference.left_axis = left.rotation().transpose() * d;
  reference.right_axis = right.rotation().transpose() * -d;
  reference.distance = 0.4;
  Eigen::MatrixXd left_jacobian = Eigen::MatrixXd::Zero(6, 12), right_jacobian = left_jacobian;
  left_jacobian.leftCols(6).setIdentity();
  right_jacobian.rightCols(6).setIdentity();
  const auto rows = facing::evaluate(left, right, left_jacobian, right_jacobian,
    left_goal, right_goal, reference, 0.02);
  ASSERT_TRUE(rows.valid);
  ASSERT_TRUE(rows.roll_valid);
  const auto moved = [](Eigen::Isometry3d pose, const Eigen::VectorXd & twist) {
      pose.translation() += twist.head<3>();
      const Eigen::Vector3d w = twist.tail<3>();
      if (w.norm() > 0.0) {
        pose.linear() = Eigen::AngleAxisd(w.norm(), w.normalized()) * pose.linear();
      }
      return pose;
    };
  const double h = 1.0e-6;
  for (int column = 0; column < 12; ++column) {
    Eigen::VectorXd step = Eigen::VectorXd::Zero(12);
    step(column) = h;
    const auto plus = facing::evaluate(moved(left, step.head(6)), moved(right, step.tail(6)), {}, {},
      left_goal, right_goal, reference, 0.02);
    const auto minus = facing::evaluate(moved(left, -step.head(6)), moved(right, -step.tail(6)), {}, {},
      left_goal, right_goal, reference, 0.02);
    const facing::Vector6 numeric = -(plus.error - minus.error) / (2.0 * h);
    EXPECT_LT((numeric - rows.jacobian.col(column)).cwiseAbs().maxCoeff(), 1.0e-4) << column;
  }
}

TEST(BimanualFacingGeometry, FeedforwardKeepsAMovingPointingReference)
{
  Eigen::Isometry3d left = Eigen::Isometry3d::Identity(), right = left;
  right.translation().x() = 0.5;
  facing::Reference from, to;
  from.left_axis = Eigen::Vector3d(1.0, 0.2, 0.1).normalized();
  from.right_axis = -Eigen::Vector3d::UnitX();
  to = from;
  to.left_axis = Eigen::Vector3d::UnitX();
  facing::ReferenceRate rate;
  const auto reference = facing::interpolate(from, to, 0.0, &rate);
  const auto rows = facing::feedforwardRows(reference, rate);
  // Rotating the left EE with the commanded rate keeps R a on the line as a moves.
  Eigen::Vector3d u1, u2;
  facing::pointingBasis(reference.left_axis, u1, u2);
  const Eigen::Vector3d omega = rows(0) * u1 + rows(1) * u2;
  const double h = 1.0e-5;
  const auto next = facing::interpolate(from, to, h);
  const Eigen::Matrix3d start = Eigen::Quaterniond::FromTwoVectors(
    reference.left_axis, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Matrix3d moved = Eigen::AngleAxisd(
    h * omega.norm(), (start * omega).normalized()).toRotationMatrix() * start;
  EXPECT_LT((moved * next.left_axis - Eigen::Vector3d::UnitX()).norm(), 1.0e-8);
}

TEST(SingleArmCartesianPlanner, PlansEitherArmWithVariableJointCountsAndKeepsOtherArmFixed)
{
  SyntheticRobot robot(4, 6, 0.6, -0.8);
  const auto & start = robot.scene->getCurrentState();
  for (const auto & arm : {robot.left, robot.right}) {
    GoalSpec goal;
    goal.reference_frame = "reference";
    const Eigen::Isometry3d base = start.getGlobalLinkTransform("reference");
    goal.relative_target = base.inverse() * start.getGlobalLinkTransform(arm.tip_frame);
    goal.relative_target.translation().x() += 0.01;
    goal.relative_position_tolerance = Eigen::Vector3d::Constant(0.0001);
    goal.relative_orientation_tolerance = Eigen::Vector3d::Constant(0.0001);
    PlannedTrajectory plan;
    std::string error;
    SingleArmCartesianPlanner planner;
    ASSERT_TRUE(planner.plan(robot.scene, start, arm, goal, robot.options(), plan, error)) << error;
    EXPECT_EQ(plan.joint_trajectory.joint_names.size(), arm.joint_names.size());
    const auto & other = arm.name == robot.left.name ? robot.right : robot.left;
    for (std::size_t i = 0; i < plan.trajectory->getWayPointCount(); ++i) {
      for (const auto & name : other.joint_names) {
        EXPECT_DOUBLE_EQ(plan.trajectory->getWayPoint(i).getVariablePosition(name), start.getVariablePosition(name));
      }
    }
    const Eigen::Isometry3d achieved = base.inverse() * plan.trajectory->getLastWayPoint().getGlobalLinkTransform(arm.tip_frame);
    EXPECT_LE((achieved.translation() - goal.relative_target.translation()).cwiseAbs().maxCoeff(), 0.000101);
    ASSERT_TRUE(planner.validatePlan(robot.scene, arm, goal, robot.options(), plan, error)) << error;
  }
}

TEST(SingleArmCartesianPlanner, RejectsCollisionBetweenCartesianWaypoints)
{
  SyntheticRobot robot;
  robot.obstacle(0.31);
  GoalSpec goal;
  goal.reference_frame = "world";
  goal.relative_target = robot.scene->getCurrentState().getGlobalLinkTransform(robot.right.tip_frame);
  goal.relative_target.translation().x() += 0.02;
  goal.cartesian_step = 0.03;
  goal.relative_position_tolerance = Eigen::Vector3d::Constant(0.00001);
  goal.relative_orientation_tolerance = Eigen::Vector3d::Constant(0.00001);
  auto options = robot.options();
  options.recovery_seeds = options.max_subdivision_depth = 0;
  PlannedTrajectory plan;
  std::string error;
  EXPECT_FALSE(SingleArmCartesianPlanner().plan(robot.scene, robot.scene->getCurrentState(),
    robot.right, goal, options, plan, error));
  EXPECT_FALSE(plan.trajectory);
}

TEST(BimanualCartesianTiming, StationaryTrajectoryGetsHoldAndZeroDerivatives)
{
  SyntheticRobot robot;
  robot_trajectory::RobotTrajectory trajectory(robot.model, robot.pair.group_name);
  auto state = robot.scene->getCurrentState();
  state.setVariableVelocity(robot.right.joint_names.front(), 0.2);
  state.setVariableAcceleration(robot.right.joint_names.front(), 0.3);
  trajectory.addSuffixWayPoint(state, 0.0);
  trajectory.addSuffixWayPoint(state, 0.0);
  std::vector<std::string> joints = robot.left.joint_names;
  joints.insert(joints.end(), robot.right.joint_names.begin(), robot.right.joint_names.end());
  std::string error;
  ASSERT_TRUE(timing::parameterize(trajectory, joints, 0.5, 0.5, error)) << error;
  EXPECT_DOUBLE_EQ(trajectory.getWayPointDurationFromPrevious(0), 0.0);
  EXPECT_DOUBLE_EQ(trajectory.getWayPointDurationFromPrevious(1), 0.1);
  for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
    for (const auto & name : joints) {
      EXPECT_DOUBLE_EQ(trajectory.getWayPoint(i).getVariableVelocity(name), 0.0);
      EXPECT_DOUBLE_EQ(trajectory.getWayPoint(i).getVariableAcceleration(name), 0.0);
    }
  }
}

TEST(BimanualCartesianPlanner, RejectsInvalidGroupsBoundsAndZeroResolution)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  auto options = robot.options();
  auto arm = robot.right;
  arm.group_name = "unknown";
  EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    arm, robot.pair, robot.goal(), options, plan, error));
  options.tightened_limits[robot.right.joint_names[0]] = {0.1, 0.2};
  EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, robot.goal(), options, plan, error));
  options = robot.options();
  options.validation_joint_step = 0.0;
  EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, robot.goal(), options, plan, error));
}

TEST(BimanualCartesianPlanner, MissingAccelerationLimitsFailRatherThanUseInventedLimits)
{
  SyntheticRobot robot;
  const auto & name = robot.right.joint_names[0];
  auto * joint = robot.model->getJointModel(name);
  auto bounds = joint->getVariableBounds(name);
  bounds.acceleration_bounded_ = false;
  joint->setVariableBounds(name, bounds);
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, robot.goal(), robot.options(), plan, error));
  EXPECT_NE(error.find("velocity/acceleration"), std::string::npos);
}

TEST(BimanualCartesianPlanner, PassiveJointInsideCommandedArmIsRejected)
{
  SyntheticRobot robot;
  robot.model->getJointModel(robot.right.joint_names[0])->setPassive(true);
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, robot.goal(), robot.options(), plan, error));
  EXPECT_NE(error.find("Passive joints"), std::string::npos);
  EXPECT_FALSE(plan.trajectory);
}

TEST(BimanualCartesianPlanner, CancellationDoesNotProduceAValidPlan)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, robot.goal(), robot.options(), plan, error, [] {return true;}));
  EXPECT_NE(error.find("canceled"), std::string::npos);
  EXPECT_FALSE(plan.trajectory);
}

TEST(BimanualCartesianPlanner, RevalidationDetectsObstacleBetweenOriginalWaypoints)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  auto goal = robot.goal(0.04);
  goal.cartesian_step = 0.04;
  auto options = robot.options();
  // A large tolerance separates the interpolation/collision check from nominal
  // progress: both original endpoints are clear of the new obstacle.
  goal.facing.distance_tolerance = 0.005;
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  const double end_x = plan.trajectory->getLastWayPoint().getGlobalLinkTransform(robot.right.tip_frame).translation().x();
  robot.obstacle(0.5 * (0.3 + end_x));
  EXPECT_FALSE(robot.scene->isStateColliding(plan.trajectory->getFirstWayPoint()));
  EXPECT_FALSE(robot.scene->isStateColliding(plan.trajectory->getLastWayPoint()));
  EXPECT_FALSE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error));
  EXPECT_NE(error.find("planning scene"), std::string::npos);
}

TEST(BimanualCartesianPlanner, RevalidationUsesLiveNonCommandedRobotJoints)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  const auto goal = robot.goal(0.02);
  const auto options = robot.options();
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  robot.scene->getCurrentStateNonConst().setVariablePosition(robot.spare_joint, -1.69);
  robot.scene->getCurrentStateNonConst().update();
  EXPECT_FALSE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error));
  EXPECT_NE(error.find("planning scene"), std::string::npos);
}

TEST(BimanualCartesianPlanner, RevalidationAllowsBaseDriftAndRejectsChangedReferences)
{
  SyntheticRobot robot(4, 6, 0.0, 0.0, true);
  BimanualCartesianPlanner planner;
  auto goal = robot.goal(0.02);
  goal.reference_frame = "reference";
  PlannedTrajectory plan;
  std::string error;
  const auto options = robot.options();
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  Eigen::Isometry3d drift = Eigen::Isometry3d::Identity();
  drift.translation() = Eigen::Vector3d(0.001, -0.002, 0.0005);
  drift.linear() = Eigen::AngleAxisd(0.002, Eigen::Vector3d::UnitY()).toRotationMatrix();
  robot.scene->getCurrentStateNonConst().setJointPositions("world_joint", drift);
  robot.scene->getCurrentStateNonConst().update();
  ASSERT_TRUE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error)) << error;
  for (int field = 0; field < 3; ++field) {
    auto changed = plan;
    auto changed_goal = goal;
    if (field == 0) {changed.facing_start.distance += 0.01;}
    if (field == 1) {changed.facing_target.roll += 0.01;}
    if (field == 2) {changed_goal.facing.distance += 0.01;}
    EXPECT_FALSE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
      changed_goal, options, changed, error));
    EXPECT_NE(error.find("facing references"), std::string::npos) << error;
  }
}

TEST(BimanualCartesianPlanner, RevalidationChecksCollisionsAtLiveFloatingBasePose)
{
  SyntheticRobot robot(4, 6, 0.0, 0.0, true);
  BimanualCartesianPlanner planner;
  auto goal = robot.goal(0.02);
  goal.reference_frame = "world";
  PlannedTrajectory plan;
  std::string error;
  const auto options = robot.options();
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  Eigen::Isometry3d drift = Eigen::Isometry3d::Identity();
  drift.translation().x() = 0.02;
  robot.scene->getCurrentStateNonConst().setJointPositions("world_joint", drift);
  robot.scene->getCurrentStateNonConst().update();
  robot.obstacle(0.32);
  EXPECT_FALSE(robot.scene->isStateColliding(plan.trajectory->getFirstWayPoint()));
  EXPECT_TRUE(robot.scene->isStateColliding(robot.scene->getCurrentState()));
  EXPECT_FALSE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error));
  EXPECT_NE(error.find("planning scene"), std::string::npos);
}

TEST(BimanualCartesianPlanner, RevalidationChecksQuinticAccelerationBetweenPoints)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  const auto goal = robot.goal(0.02);
  const auto options = robot.options();
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  for (auto & point : plan.joint_trajectory.points) {
    const double old = point.time_from_start.sec + point.time_from_start.nanosec * 1.0e-9;
    const auto ns = static_cast<int64_t>(old * 0.01 * 1.0e9);
    point.time_from_start.sec = static_cast<int32_t>(ns / 1000000000LL);
    point.time_from_start.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    for (auto & value : point.velocities) {value *= 100.0;}
    for (auto & value : point.accelerations) {value *= 10000.0;}
  }
  EXPECT_FALSE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error));
  EXPECT_NE(error.find("bounds"), std::string::npos);
}

TEST(SingleArmCartesianPlanner, FloatingBaseOnlyMotionRespectsLockedAxesAndAllArmJoints)
{
  SyntheticRobot robot(4, 4, 0.0, 0.0, true);
  GoalSpec goal;
  goal.reference_frame = "world_ned";
  goal.relative_target = robot.scene->getCurrentState().getGlobalLinkTransform(robot.right.tip_frame);
  goal.relative_target.translation().x() += 0.03;
  goal.relative_position_tolerance.setConstant(0.00001);
  goal.relative_orientation_tolerance.setConstant(0.00001);
  goal.available_dofs.assign(10, false);
  goal.available_dofs[0] = true;
  goal.base_lower_limits = {-0.1, -0.1, -0.1, -0.1, -0.1, -0.1};
  goal.base_upper_limits = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  auto options = robot.options();
  options.base_frame = "world";
  PlannedTrajectory plan;
  std::string error;
  SingleArmCartesianPlanner planner;
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.right,
    goal, options, plan, error)) << error;
  ASSERT_EQ(plan.base_trajectory.points.size(), plan.joint_trajectory.points.size());
  ASSERT_TRUE(planner.validatePlan(robot.scene, robot.right, goal, options, plan, error)) << error;
  const double total = timing::seconds(plan.base_trajectory.points.back().time_from_start);
  for (int i = 0; i <= 80; ++i) {
    const double time = total * i / 80;
    BaseTrajectorySample sample;
    ASSERT_TRUE(sampleBaseTrajectory(plan.base_trajectory, time, sample, error)) << error;
    EXPECT_LE(std::abs(sample.coordinates(0)), 0.100001);
    EXPECT_LE(sample.velocity.cwiseAbs().maxCoeff(), 0.150001);
    EXPECT_LE(sample.acceleration.cwiseAbs().maxCoeff(), 0.150001);
    for (Eigen::Index axis = 1; axis < 6; ++axis) {EXPECT_NEAR(sample.coordinates(axis), 0.0, 1.0e-9);}
    trajectory_msgs::msg::JointTrajectoryPoint arm;
    ASSERT_TRUE(BimanualCartesianPlanner::sampleTrajectory(plan.joint_trajectory, time, arm, error));
    for (const double value : arm.positions) {EXPECT_NEAR(value, 0.0, 1.0e-9);}
  }
  const auto final = baseCoordinates(plan.trajectory->getLastWayPoint().getGlobalLinkTransform("world"));
  EXPECT_NEAR(final(0), 0.03, 0.000011);
}

TEST(SingleArmCartesianPlanner, EmptyMaskProducesStationaryBaseAndMovableArm)
{
  SyntheticRobot robot(4, 4, 0.0, 0.0, true);
  GoalSpec goal;
  goal.reference_frame = "world_ned";
  goal.relative_target = robot.scene->getCurrentState().getGlobalLinkTransform(robot.left.tip_frame);
  goal.relative_target.translation().x() += 0.02;
  goal.relative_position_tolerance.setConstant(0.00001);
  goal.relative_orientation_tolerance.setConstant(0.00001);
  auto options = robot.options();
  options.base_frame = "world";
  PlannedTrajectory plan;
  std::string error;
  ASSERT_TRUE(SingleArmCartesianPlanner().plan(robot.scene, robot.scene->getCurrentState(),
    robot.left, goal, options, plan, error)) << error;
  for (const auto & point : plan.base_trajectory.points) {
    EXPECT_DOUBLE_EQ(point.transforms.front().translation.x, 0.0);
    EXPECT_DOUBLE_EQ(point.transforms.front().translation.y, 0.0);
    EXPECT_DOUBLE_EQ(point.transforms.front().translation.z, 0.0);
  }
  EXPECT_GT(plan.joint_trajectory.points.back().positions.front(), 0.01998);
}

TEST(SingleArmCartesianPlanner, RejectsInvalidMaskAndBaseLimitViolation)
{
  SyntheticRobot robot(4, 4, 0.0, 0.0, true);
  GoalSpec goal;
  goal.reference_frame = "world_ned";
  goal.relative_target = robot.scene->getCurrentState().getGlobalLinkTransform(robot.right.tip_frame);
  goal.relative_target.translation().x() += 0.03;
  goal.available_dofs.assign(9, true);
  auto options = robot.options();
  options.base_frame = "world";
  PlannedTrajectory plan;
  std::string error;
  SingleArmCartesianPlanner planner;
  EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.right,
    goal, options, plan, error));
  EXPECT_NE(error.find("available_dofs"), std::string::npos);
  goal.available_dofs.assign(10, false);
  goal.available_dofs[0] = true;
  goal.base_upper_limits = {0.01, 1.0, 1.0, 1.0, 1.0, 1.0};
  EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.right,
    goal, options, plan, error));
}

}  // namespace
}  // namespace sura_manipulation_actions::bimanual_cartesian
