# Automatically Generated in 2017-06-12 22:33:47.452922
# MESSAGE: ODOMETER_DELTA
# Description:Measurement of Odometry Sensor within a time-period
uint8 ID = 138
uint8 sysid
uint8 compid

Header header

float32 delta                   # Time-delta within which the measurement has taken place [s]
geometry_msgs/Vector3 dist      # Distance travelled within period [m]
geometry_msgs/Vector3 velocity  # Velocity [m/s]
int16 quality                   # Measurement quality indicator (-1 for no quality)
