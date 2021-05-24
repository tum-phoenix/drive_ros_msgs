#Left=0, Right=1, Undefined=2
#   |   |   |
#   2       |
#   |   |   |
#   |       |
# 2 | 0 | 1 | 2
#   |       |
#   |   2   
#   |        
#   |   |   |_|
#   |       |2|
#   |   |    /
#   |       /
#   |   |   |
int8 LEFT=0
int8 RIGHT=1
int8 UNDEFINED=2
int8 current_lane

float32 front_distance

drive_ros_msgs/ObstacleEnvironment[] obstacles

#Uses the same mapping as objects
drive_ros_msgs/PedestrianEnvironment[] pedestrians

drive_ros_msgs/TrafficMarkEnvironment[] traffic_marks

#missing: parking spots