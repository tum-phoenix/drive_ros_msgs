# Automatically Generated in 2017-06-12 22:33:47.452851
# MESSAGE: IMU
# Description: Measurement of 9DOF Inertial Measurement Unit (IMU)
Header header

uint8 ID = 128
uint8 sysid
uint8 compid

geometry_msgs/Vector3 acc       # Linear acceleration [g]
geometry_msgs/Vector3 gyro      # Angular velocity [rad/s]
geometry_msgs/Vector3 mag       # Magnetic field strength [T]
