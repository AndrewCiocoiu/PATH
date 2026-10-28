# Where the barrel must point to hit the target. Published on /aim/solution by aim_solver_node (stage S5).
std_msgs/Header header

float64 yaw_cmd          # [rad] base_link
float64 pitch_cmd        # [rad] base_link (includes ballistic elevation)
float64 time_of_flight   # [s]
float64 aim_error        # [rad] angle between current barrel direction and this solution
bool valid
