cmake_minimum_required(VERSION 3.5)
project(drive_ros_msgs)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(
    sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)

ament_export_include_directories(
    "include"
 )

rosidl_generate_interfaces(
    ${PROJECT_NAME}
    "msg/RoadLane.msg"
    "msg/RoadLine.msg"
    "msg/Homography.msg"
    "msg/Obstacle.msg"
    "msg/ObstacleArray.msg"
    "msg/TimeCompare.msg"
    "msg/EncoderLinear.msg"
    "msg/EnvironmentModel.msg"
    "msg/ObstacleEnvironment.msg"
    "msg/PedestrianEnvironment.msg"
    "msg/TrafficMarkEnvironment.msg"
    "msg/Lane.msg"
    "msg/TrajectoryMetaInput.msg"
    "msg/VehicleEncoder.msg"
    "msg/BoundingBox.msg"
    "msg/BoundingBoxArray.msg"
    "msg/SimpleTrajectory.msg"
    "msg/DrivingLine.msg"
    "msg/DetectedIntersection.msg"
    "msg/StreetMarking.msg"
    "msg/StreetMarkingArray.msg"
    "msg/TrajectoryPoint.msg"
    "msg/Trajectory.msg"
    "srv/ParkingInProgress.srv"
    DEPENDENCIES sensor_msgs std_msgs
)

# add header files to ros
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_package()