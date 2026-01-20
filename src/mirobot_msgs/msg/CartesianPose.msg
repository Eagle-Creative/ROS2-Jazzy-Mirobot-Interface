# Cartesian pose for the Mirobot end-effector
# Position in millimeters, orientation in degrees

# Position
float64 x
float64 y
float64 z

# Orientation (Roll, Pitch, Yaw)
float64 roll
float64 pitch
float64 yaw

# Movement mode: 0 = point-to-point (G0), 1 = linear (G1)
uint8 MODE_PTP = 0
uint8 MODE_LINEAR = 1
uint8 mode

# Speed in mm/s
float64 speed
