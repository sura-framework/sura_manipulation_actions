#!/usr/bin/env python3
"""Send an end-effectors facing goal; the distance defaults to the current one from TF."""

import argparse
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sura_manipulation_actions.action import PlanBimanualCartesianTrajectory
from tf2_ros import Buffer, TransformException, TransformListener


AXES = {"+x": (1.0, 0.0, 0.0), "-x": (-1.0, 0.0, 0.0), "+y": (0.0, 1.0, 0.0),
        "-y": (0.0, -1.0, 0.0), "+z": (0.0, 0.0, 1.0), "-z": (0.0, 0.0, -1.0)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-namespace", default="cirtesub")
    parser.add_argument("--left-tip", default="")
    parser.add_argument("--right-tip", default="")
    parser.add_argument("--left-axis", choices=AXES, default="+x",
                        help="Left tool axis that points at the right tool (--left-axis=-x for negative)")
    parser.add_argument("--right-axis", choices=AXES, default="+x",
                        help="Right tool axis that points at the left tool")
    parser.add_argument("--roll", type=float, default=float("nan"),
                        help="Relative roll [rad]; omitted keeps the measured roll")
    parser.add_argument("--distance", type=float, default=0.0,
                        help="Distance between tools [m]; zero uses the current distance plus --delta")
    parser.add_argument("--delta", type=float, default=0.0,
                        help="Change of the current distance [m] when --distance is zero")
    parser.add_argument("--action", default="")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--velocity-scaling", type=float, default=0.0,
                        help="Joint velocity limit factor in (0,1]; zero uses server default")
    parser.add_argument("--acceleration-scaling", type=float, default=0.0,
                        help="Joint acceleration limit factor in (0,1]; zero uses server default")
    args, ros_args = parser.parse_known_args()
    for name in ("velocity_scaling", "acceleration_scaling"):
        value = getattr(args, name)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            parser.error(f"--{name.replace('_', '-')} must be finite in [0,1]")
    rclpy.init(args=ros_args)
    node = Node("bimanual_cartesian_example")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    namespace = args.robot_namespace.strip("/")
    frame_prefix = f"{namespace}/" if namespace else ""
    topic_prefix = f"/{namespace}" if namespace else ""
    left_tip = args.left_tip or f"{frame_prefix}alpha_left/standard_jaws_tool"
    right_tip = args.right_tip or f"{frame_prefix}alpha_right/standard_jaws_tool"
    action_name = args.action or f"{topic_prefix}/manipulation/plan_bimanual_cartesian_trajectory"
    client = ActionClient(node, PlanBimanualCartesianTrajectory, action_name)
    try:
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"Action unavailable: {action_name}; configure the lifecycle node")
        distance = args.distance
        if distance <= 0.0:
            future = buffer.wait_for_transform_async(left_tip, right_tip, Time())
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            if not future.done():
                raise RuntimeError("Timed out waiting for the TF between both tools")
            offset = buffer.lookup_transform(left_tip, right_tip, Time()).transform.translation
            distance = math.hypot(offset.x, offset.y, offset.z) + args.delta
        goal = PlanBimanualCartesianTrajectory.Goal()
        for vector, name in ((goal.left_pointing_axis, args.left_axis),
                             (goal.right_pointing_axis, args.right_axis)):
            vector.x, vector.y, vector.z = AXES[name]
        goal.roll_angle = args.roll
        goal.distance = distance
        goal.pointing_tolerance = 0.05
        goal.roll_tolerance = 0.05
        goal.distance_tolerance = 0.005
        goal.cartesian_step = 0.005
        goal.planning_time = 10.0
        goal.velocity_scaling = args.velocity_scaling
        goal.acceleration_scaling = args.acceleration_scaling
        goal.execute = args.execute
        future = client.send_goal_async(
            goal, feedback_callback=lambda msg: node.get_logger().info(
                f"{msg.feedback.current_stage}: {msg.feedback.message}"))
        rclpy.spin_until_future_complete(node, future)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError("Cartesian goal rejected; inspect the server diagnostic")
        node.get_logger().info(
            "Goal accepted. With execute=false it stays pending until the execute service is called.")
        result = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(node, result)
        except KeyboardInterrupt:
            canceled = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, canceled, timeout_sec=2.0)
            return
        node.get_logger().info(result.result().result.message)
    except (RuntimeError, TransformException) as error:
        node.get_logger().error(str(error))
    finally:
        del listener
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
