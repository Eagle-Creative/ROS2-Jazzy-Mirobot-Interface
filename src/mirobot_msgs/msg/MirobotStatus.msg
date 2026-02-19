# Mirobot Status Message
# Provides the current state of the Mirobot robot

# Header with timestamp
std_msgs/Header header

# Robot name/identifier
string robot_name

# Robot state: "idle", "busy", "homed", "error"
string state

# Gripper state: true = open, false = closed
bool gripper_open

# Additional status information
bool is_homed
string error_message
