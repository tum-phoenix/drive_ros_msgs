Header header

# possible obstacle types
uint8 TYPE_GENERIC = 0
uint8 TYPE_CAMERA = 1
uint8 TYPE_LIDAR = 2
uint8 obstacle_type

# points belonging to the obstacle (do we really need this?)
geometry_msgs/Polygon polygon

# center point
geometry_msgs/PoseWithCovariance centroid_pose
geometry_msgs/TwistWithCovariance centroid_twist

# dimensions of obstacle in rotated pose (0 if unknown)
float32 length
float32 width
float32 height

# trust value associated with obstacle
float32 trust
