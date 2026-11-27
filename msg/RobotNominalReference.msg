# One whole-body nominal reference for the Task Priority robot_nominal task.
# The pose and twist describe the base origin in header.frame_id (the controller world
# frame). base_twist.angular is the spatial angular velocity in world axes.
std_msgs/Header header
geometry_msgs/Pose base_pose
geometry_msgs/Twist base_twist
string[] joint_names
float64[] positions
float64[] velocities
# Optional mask for this reference: 6 base rows [x, y, z, roll, pitch, yaw] followed by
# every joint in controller order. Empty keeps the configured activation; false removes
# a row while the reference is live (it never enables a disabled one). With all six base
# rows false the base hold tasks are not retargeted either.
bool[] activation
