# Empty global_progress selects the ordinary whole-trajectory STPIK check.
float64[] global_progress
float64 start_progress
float64 maximum_progress
LocalSegment incumbent
float64 incumbent_elapsed
moveit_msgs/PlanningScene scene
string robot_description
string robot_description_semantic
bool explore true
# Normalized geometric advance relative to the global nominal cruising rate.
float64 progress_rate_scale 1.0
float64 horizon 0.5
float64 budget 1.0
float64 retreat_window 0.05
float64 gain 0.5
float64 amplitude 0.2
float64 position_scale 0.1
float64 angle_scale 0.5
float64 recovery_threshold 0.15
float64 improvement 0.02
# Configuration, Cartesian, terminal, inverse clearance, smoothness, exploration.
float64[6] weights [1.0, 1.0, 2.0, 0.01, 0.01, 0.01]
