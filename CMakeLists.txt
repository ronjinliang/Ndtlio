cmake_minimum_required(VERSION 3.10)
project(2dNdtLIO)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O1 -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(locsensor REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
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

set(G2O_LIBRARY_DIR ${PROJECT_SOURCE_DIR}/../../3rdparty/src/g2o/lib)
set(g2o_libs
    ${G2O_LIBRARY_DIR}/libg2o_stuff.so
    ${G2O_LIBRARY_DIR}/libg2o_core.so
    ${G2O_LIBRARY_DIR}/libg2o_solver_cholmod.so
    ${G2O_LIBRARY_DIR}/libg2o_solver_dense.so
    ${G2O_LIBRARY_DIR}/libg2o_solver_csparse.so
    ${G2O_LIBRARY_DIR}/libg2o_csparse_extension.so
    ${G2O_LIBRARY_DIR}/libg2o_types_sba.so
    ${CSPARSE_LIBRARY}
    ${CHOLMOD_LIBRARY}
)
find_package(Cholmod REQUIRED)   # g2o 需要
include_directories(${CHOLMOD_INCLUDE_DIR})

include_directories(${PROJECT_SOURCE_DIR}/..)
include_directories(${PROJECT_SOURCE_DIR}/../../3rdparty/src)   # 3rdparty的头文件

add_subdirectory(${PROJECT_SOURCE_DIR}/src)      # LIO
add_subdirectory(${PROJECT_SOURCE_DIR}/common)   # io utils

# 测试文件
add_executable(test_2d_lo test_2d_lo.cpp)
target_link_libraries(test_2d_lo
    gflags
    NDT_LIO
    io_utils
)

# ROS2 测试节点
add_library(ndt_lio_node ./ros2node/ndt_lio_node.cpp)
target_link_libraries(ndt_lio_node
    NDT_LIO
    ${YAML_CPP_LIBRARIES}  # 不知道为什么, 突然又需要链接 yaml, 之前不需要
)
ament_target_dependencies( ndt_lio_node
    sensor_msgs
    tf2_ros
    geometry_msgs
    nav_msgs
    origincar_msg
)

add_executable(test_ros2_lio test_ros2_lio.cpp)
target_link_libraries(test_ros2_lio
    gflags
    ndt_lio_node
)
