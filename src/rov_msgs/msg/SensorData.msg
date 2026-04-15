# Fused IMU + depth sensor data
# Replaces synchronizer/Combined

std_msgs/Header header

# Roll, Pitch, Yaw (degrees)
float32[3] orientation

# X, Y, Z accelerations
float32[3] acceleration

# Depth in meters
float32 depth
