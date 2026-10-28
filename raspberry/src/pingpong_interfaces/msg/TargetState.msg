# Filtered target estimate. Published on /target/state by target_tracker_node (stage S4).
# All angles are in base_link (the fixed tripod frame), see doc section 8.
std_msgs/Header header

float64 yaw_bearing     # [rad]
float64 pitch_bearing   # [rad]
float64 yaw_rate        # [rad/s]
float64 pitch_rate      # [rad/s]
float64 range           # [m]
bool range_valid
geometry_msgs/Point position   # [m] in base_link
float32 confidence      # 0 = no target / lost
uint32 track_id
uint32 track_age        # number of frames the target has been tracked
