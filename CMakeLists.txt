cmake_minimum_required(VERSION 3.10)
project(cev_localization LANGUAGES CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# CEV
find_package(cev_msgs REQUIRED)

# Add subdirectory for the library
add_subdirectory(cev_kalman_filter)

# Include directories for project headers
include_directories(
    include
    cev_kalman_filter/include
)

# Source files in the main src folder
set(SOURCES
    src/localization.cpp
    src/config_parser.cpp
    src/ros_sensor.cpp
    src/std_ros_sensors.cpp
)

# Declare the executable
add_executable(localization ${SOURCES})

# Link libraries
ament_target_dependencies(localization
    rclcpp
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    nav_msgs
    cev_msgs
    Eigen3
)

target_link_libraries(localization cev_kalman_filter yaml-cpp)

# Install targets
install(TARGETS localization
    DESTINATION lib/${PROJECT_NAME})

# Install additional resources
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rosbag
    DESTINATION share/${PROJECT_NAME})

# Ament package configuration
ament_package()
