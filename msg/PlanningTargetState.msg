# One authoritative editable target. In bimanual mode the target is the end-effectors
# facing described by the pointing axes, roll and distance (target_pose stays identity),
# and tool_poses show the left and right tools already facing each other.
bool bimanual
string server_id
uint64 revision
string marker_namespace
string frame_id
string[] arm_names
string[] tool_frames
geometry_msgs/Pose[] tool_poses
geometry_msgs/PoseStamped target_pose
# Bimanual only: pure signed axes of each tool, roll [rad] (NaN keeps the measured one)
# and distance [m].
geometry_msgs/Vector3 left_pointing_axis
geometry_msgs/Vector3 right_pointing_axis
float64 roll_angle
float64 distance
bool visible
bool locked
string owner_id
string action_name
uint8[16] goal_id
string status
