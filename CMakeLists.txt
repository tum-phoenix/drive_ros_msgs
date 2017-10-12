cmake_minimum_required(VERSION 2.8.3)
project(drive_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
    sensor_msgs
    std_msgs
    geometry_msgs
    message_generation
    message_filters
)

add_message_files(
    FILES
    RoadLane.msg
    Homography.msg
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    )

catkin_package(
    CATKIN_DEPENDS  sensor_msgs std_msgs message_runtime
)
