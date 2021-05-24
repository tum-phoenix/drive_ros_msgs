#Port of RoadLane from LMS:
std_msgs/Header header
# lane points
geometry_msgs/PointStamped[] points
# type of lane
uint8 UNKNOWN=0
uint8 STRAIGHT=1
uint8 STRAIGHT_CURVE=2
uint8 CURVE = 3
uint8 road_state_type
