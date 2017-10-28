# Automatically Generated in 2017-06-12 22:33:47.452972
# MESSAGE: ODOMETER
# Description:Measurement of Odometry Sensor
uint8 ID = 140
uint8 sysid
uint8 compid

Header header

float32 time_delta                  # Time-delta within which the measurement has taken place [s]
geometry_msgs/Vector3 dist_delta    # Distance travelled within period [m]
geometry_msgs/Vector3 dist_abs      # Distance travelled [m]
geometry_msgs/Vector3 velocity      # Velocity [m/s]
int16 quality                       # Measurement quality indicator (-1 for no quality)
