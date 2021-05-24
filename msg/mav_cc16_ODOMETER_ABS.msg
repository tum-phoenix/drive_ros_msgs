# Automatically Generated in 2017-06-12 22:33:47.452879
# MESSAGE: ODOMETER_ABS
# Description:Measurement of Odometry Sensor
std_msgs/Header header

uint8 ID = 136
uint8 sysid
uint8 compid

geometry_msgs/Vector3 dist          # Distance travelled [m]
geometry_msgs/Vector3 velocity      # Velocity [m/s]

int16 quality                       # Measurement quality indicator (-1 for no quality)
