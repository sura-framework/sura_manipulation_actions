# Streamed reference for task_priority_kinematic_control/EndEffectorsFacingTask.
std_msgs/Header header
# Pure signed goal axes (+-x, +-y, +-z); they define the zero of the roll.
geometry_msgs/Vector3 left_goal_axis
geometry_msgs/Vector3 right_goal_axis
# Current unit reference pointing axes, each expressed in its own end-effector frame.
geometry_msgs/Vector3 left_pointing_axis
geometry_msgs/Vector3 right_pointing_axis
float64 roll_angle
float64 distance
# Time derivatives (feedforward). They expire after the task's feedforward timeout.
geometry_msgs/Vector3 left_pointing_rate
geometry_msgs/Vector3 right_pointing_rate
float64 roll_rate
float64 distance_rate
