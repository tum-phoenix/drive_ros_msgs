#Left=0, Right=1, Undefined=2
#   |   |   |
#   2       |
#   |   |   |
#   |       |
# 2 | 0 | 1 | 2
#   |       |
#   |   2   \
#   |        \
#   |   |   |_|
#   |       |2|
#   |   |    /
#   |       /
#   |   |   |
int32 current_lane

#Left=0, Right=1, Left side=2, Right side=3
#Visually:
#   |   |   |
#   |       |
#   |   |   |
#   |       |
# 2 | 0 | 1 | 3
#   |       |
#   |   |   |
#   |       |
#   |   |   |
int32[] obj_lane
float32[] obj_track_distance
float32[] obj_lateral_offset #positive=obj is on the right

#Uses the same mapping as objects
int32[] pedestrian_lane

int32[] traffic_marks_id
float32[] traffic_marks_track_distance

#missing: parking spots