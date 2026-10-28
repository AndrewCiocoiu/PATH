# Distance to the target along the barrel axis. Published on /range/target.
# Any range sensor (radar, lidar, ultrasonic, image-size estimate) publishes this same message.
std_msgs/Header header

float32 range          # [m]
bool valid
float32 field_of_view  # [rad]
string sensor_id
