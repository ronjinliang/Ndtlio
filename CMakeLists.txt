cmake_minimum_required(VERSION 3.10)
project(NdtLIO)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(origincar_msg REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(TBB REQUIRED)

find_package(yaml-cpp REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/../3rdparty/src)   # 3rdparty的头文件

add_subdirectory(${PROJECT_SOURCE_DIR}/src)      # LIO

# ROS2 测试节点
add_library(ndt_lio_node ./ros2node/ndt_lio_node.cpp)
target_link_libraries(ndt_lio_node
    NDT_LIO
    ${YAML_CPP_LIBRARIES}  # 不知道为什么, 突然又需要链接 yaml, 之前不需要
)
ament_target_dependencies( ndt_lio_node
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    origincar_msg
)

add_executable(test_ros2_lio test_ros2_lio.cpp)
target_link_libraries(test_ros2_lio
    ndt_lio_node
)

# 获取IMU信息的节点
add_executable(process_imu process_imu.cpp)
target_link_libraries(process_imu
  glog
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies( process_imu
  rclcpp
  sensor_msgs
)

install(TARGETS ndt_lio_node test_ros2_lio process_imu
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()