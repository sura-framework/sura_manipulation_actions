# Path of the end-effectors facing reference, parametrized by progress.
# Pure signed goal axes (+-x, +-y, +-z); they also define the zero of the roll.
geometry_msgs/Vector3 left_goal_axis
geometry_msgs/Vector3 right_goal_axis
FacingPathPoint[] points
