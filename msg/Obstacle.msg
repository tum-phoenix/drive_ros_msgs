Header header
# possible obstacle types
uint8 TYPE_GENERIC = 0
uint8 TYPE_CAMERA = 1
uint8 TYPE_LIDAR = 2
uint8 obstacle_type
geometry_msgs/Polygon polygon
geometry_msgs/Point centroid
float32 width
float32 trust
