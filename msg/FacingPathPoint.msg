# One knot of a facing path. progress is the geometric parameter in [0, 1]; no timing.
float64 progress
# Unit reference pointing axes, each expressed in its own end-effector frame.
geometry_msgs/Vector3 left_pointing_axis
geometry_msgs/Vector3 right_pointing_axis
# Relative roll around the line left EE -> right EE [rad] and distance [m].
float64 roll_angle
float64 distance
