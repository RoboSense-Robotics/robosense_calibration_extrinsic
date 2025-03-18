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
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

ament_package_xml()
set(ROS_DISTRO "ROS 2")

set(${PROJECT_NAME}_source_files
        src/common/common_function.cpp
        src/calibration/base_calibration.cpp
        src/calibration/sensor_calibration.cpp)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_source_files})

set_target_properties(
  ${PROJECT_NAME}
  PROPERTIES CXX_STANDARD 14
             CXX_STANDARD_REQUIRED YES
             CXX_EXETENSIONS NO)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC include)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
        ${EIGEN3_INCLUDE_DIRS} 
        ${Boost_INCLUDE_DIRS} 
        ${YAML_CPP_INCLUDE_DIRS} 
        ${PCL_INCLUDE_DIRS} 
        ${pcl_ros_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS} 
        ${sensor_msgs_INCLUDE_DIRS}
        ${pcl_conversions_INCLUDE_DIRS}
         )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        yaml-cpp
        ${cv_bridge_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        ${pcl_conversions_LIBRARIES}
        ${pcl_ros_LIBRARIES}
        ${rclcpp_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        )

# 添加可执行文件
add_executable(sensor_to_body_calibration_node node/sensor_to_body_calibration_node.cpp)

# 链接库
target_link_libraries(sensor_to_body_calibration_node ${PROJECT_NAME})

ament_target_dependencies(sensor_to_body_calibration_node
  yaml-cpp
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
      )

install(TARGETS
  ${PROJECT_NAME}
  sensor_to_body_calibration_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
