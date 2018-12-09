cmake_minimum_required(VERSION 2.8.3)
project(drive_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
    sensor_msgs
    std_msgs
    geometry_msgs
    message_generation
    message_filters
    tf2_geometry_msgs
)

include_directories(
    "include"
 )

add_message_files(
    FILES
    RoadLane.msg
    RoadLine.msg
    Homography.msg
    Obstacle.msg
    ObstacleArray.msg
    TimeCompare.msg
    EncoderLinear.msg
    VehicleEncoder.msg
    BoundingBox.msg
    BoundingBoxArray.msg
    simple_trajectory.msg
    DrivingLine.msg

    mav_RAW_DATA.msg
    mav_cc16_NOTIFICATION.msg
    mav_cc16_HEARTBEAT.msg
    mav_cc16_DEBUG.msg
    mav_cc16_TELEMETRY.msg
    mav_cc16_CONTROL_LIGHTS.msg
    mav_cc16_CONTROL_COMMAND.msg
    mav_cc16_IMU.msg
    mav_cc16_ODOMETER_ABS.msg
    mav_cc16_ODOMETER_RAW.msg
    mav_cc16_ODOMETER_DELTA.msg
    mav_cc16_ODOMETER_DELTA_RAW.msg
    mav_cc16_ODOMETER.msg
    mav_cc16_PROXIMITY.msg
    mav_cc16_PARKING_LOT.msg
    mav_cc16_CONFIG_REQUEST_COUNT.msg
    mav_cc16_CONFIG_REQUEST.msg
    mav_cc16_CONFIG_REQUEST_PARAMS.msg
    mav_cc16_CONFIG_COUNT.msg
    mav_cc16_CONFIG.msg
    mav_cc16_CONFIG_PARAM_INT.msg
    mav_cc16_CONFIG_PARAM_BOOL.msg
    mav_cc16_CONFIG_PARAM_FLOAT.msg
    mav_cc16_CONFIG_PARAM_SET_INT.msg
    mav_cc16_CONFIG_PARAM_SET_BOOL.msg
    mav_cc16_CONFIG_PARAM_SET_FLOAT.msg
    mav_cc16_COMMAND.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)


catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS  sensor_msgs std_msgs message_runtime
)

# add header files to ros
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
