Header header

# possible obstacle types
uint8 TYPE_GENERIC = 0
uint8 TYPE_CAMERA = 1
uint8 TYPE_LIDAR = 2
uint8 obstacle_type

# points belonging to the obstacle
geometry_msgs/Polygon polygon

# center point of the points
geometry_msgs/Point32 centroid

# orientation of the object
geometry_msgs/Quaternion orientation

# dimensions of obstacle in rotated pose (0 if unknown)
float32 length
float32 width
float32 height

# trust value associated with obstacle
float32 trust
