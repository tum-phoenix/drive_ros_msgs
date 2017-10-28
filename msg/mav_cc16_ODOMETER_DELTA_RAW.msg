# Automatically Generated in 2017-06-12 22:33:47.452947
# MESSAGE: ODOMETER_DELTA_RAW
# Description:Measurement of Odometry Sensor within a time-period (raw ticks)
uint8 ID = 139
uint8 sysid
uint8 compid

Header header

float32 delta                   # Time-delta within which the measurement has taken place [s]
geometry_msgs/Vector3 dist      # Distance travelled within period [ticks]
int16 quality                   # Measurement quality indicator (-1 for no quality)
