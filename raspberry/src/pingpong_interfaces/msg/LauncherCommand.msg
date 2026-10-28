# Command to the launcher. Published on /launcher/cmd on events.
# The ESP32 ignores a repeated seq, so a FIRE is never executed twice.
uint8 LOAD=0
uint8 COCK=1
uint8 FIRE=2
uint8 UNLOAD=3
uint8 RESET=4

std_msgs/Header header
uint8 command
uint32 seq
