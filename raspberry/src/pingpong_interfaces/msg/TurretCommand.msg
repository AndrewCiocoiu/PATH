# Command to the yaw/pitch axes. Published on /turret/cmd at 50 Hz.
# It is also the HEARTBEAT: if the ESP32 receives nothing valid for 250 ms it stops and blocks firing.
uint8 HOLD=0
uint8 POSITION=1
uint8 VELOCITY=2
uint8 HOME=3

std_msgs/Header header
uint8 mode
float64 yaw          # [rad]   used in POSITION mode
float64 pitch        # [rad]   used in POSITION mode
float64 yaw_rate     # [rad/s] used in VELOCITY mode (feed-forward in POSITION mode)
float64 pitch_rate   # [rad/s]
uint32 seq
