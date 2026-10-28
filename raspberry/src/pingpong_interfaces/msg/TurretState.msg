# Measured turret state from the ESP32. Published on /turret/state.
# header.stamp = time of MEASUREMENT (needed by the tracker, doc section 8.2).
std_msgs/Header header

float64 yaw          # [rad]
float64 pitch        # [rad]
float64 yaw_rate     # [rad/s]
float64 pitch_rate   # [rad/s]
bool homed
bool at_limit
bool moving
