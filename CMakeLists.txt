cmake_minimum_required(VERSION 3.5)

project(robosense_calibration_extrinsic LANGUAGES CXX)

# 设置自动MOC、RCC和UIC
set(CMAKE_AUTORCC ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Qt5 COMPONENTS Core)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(ROS_DISTRO "ROS 2")

set(dependencies
    rclcpp
    pcl_conversions
    pcl_ros
    sensor_msgs
    nav_msgs
    geometry_msgs
)

set(${PROJECT_NAME}_source_files
        src/common/common_function.cpp
        src/calibration/base_calibration.cpp
        src/calibration/sensor_calibration.cpp
        src/calibration/robotic_arm_calibration.cpp)

add_library(${PROJECT_NAME}_lib ${${PROJECT_NAME}_source_files})

target_include_directories(${PROJECT_NAME}_lib PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${pcl_ros_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${pcl_conversions_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_lib
        ${YAML_CPP_LIBRARIES}
        ${cv_bridge_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        ${pcl_conversions_LIBRARIES}
        ${pcl_ros_LIBRARIES}
        ${rclcpp_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME}_lib PUBLIC -DPROJECT_PATH="${CMAKE_CURRENT_LIST_DIR}/")
ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})

add_executable(sensor_to_arm_calibration_node node/sensor_to_arm_calibration_node.cpp)
target_link_libraries(sensor_to_arm_calibration_node ${PROJECT_NAME}_lib)
ament_target_dependencies(sensor_to_arm_calibration_node ${dependencies})

add_executable(sensor_to_body_calibration_node node/sensor_to_body_calibration_node.cpp)
target_link_libraries(sensor_to_body_calibration_node ${PROJECT_NAME}_lib)
ament_target_dependencies(sensor_to_body_calibration_node ${dependencies})

add_executable(test_node node/hand_eye_test.cpp)
target_link_libraries(test_node ${PROJECT_NAME}_lib)
ament_target_dependencies(test_node ${dependencies})

install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
      )

install(TARGETS
  ${PROJECT_NAME}_lib
  sensor_to_body_calibration_node
  sensor_to_arm_calibration_node
  test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()