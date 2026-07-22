cmake_minimum_required(VERSION 3.10)
project(2dNdtLIO)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O1 -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(velodyne_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(angles REQUIRED)
find_package(locsensor REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(origincar_msg REQUIRED)

find_package(PCL REQUIRED COMPONENTS 
    common io filters kdtree search sample_consensus visualization
)

include_directories(${PCL_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# find_package(Pangolin REQUIRED)
# include_directories(${Pangolin_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# find_package(TBB REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/..)
include_directories(${PROJECT_SOURCE_DIR}/../../3rdparty/src)   # 3rdparty的头文件

find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})

# 设置 g2o 库路径 - 修复路径和库名称
set(G2O_LIBRARY_DIR ${PROJECT_SOURCE_DIR}/../../3rdparty/src/g2o/lib)
# 设置RPATH策略：优先使用构建目录和安装目录中的库
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
# 将构建时库目录（您的本地g2o）添加到RPATH
set(CMAKE_BUILD_RPATH ${G2O_LIBRARY_DIR})
# 同时添加项目自身的库目录（如liblikelihood_field.so所在目录）
set(CMAKE_BUILD_RPATH "${CMAKE_BUILD_RPATH};${CMAKE_CURRENT_BINARY_DIR}")
# 确保安装后的可执行文件也能找到库
set(CMAKE_INSTALL_RPATH ${G2O_LIBRARY_DIR})
# 不添加系统默认库路径（避免ROS系统库干扰）
set(CMAKE_SKIP_RPATH FALSE)
set(CMAKE_SKIP_INSTALL_RPATH FALSE)

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

set(libcommon ${PROJECT_SOURCE_DIR}/../common/lib/libcommon.so)
set(libtools ${PROJECT_SOURCE_DIR}/../tools/pointcloud_convert/build/libtools.so)


add_library(lidar_2d_utils lidar_2d_utils.cpp)
target_link_libraries(lidar_2d_utils
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
ament_target_dependencies(lidar_2d_utils
    rclcpp
    rosbag2_cpp
    sensor_msgs
    velodyne_msgs
    pcl_conversions
    livox_ros_driver2
)

add_library(ndt_inc ndt_inc.cpp)
target_link_libraries(ndt_inc
    glog
)
ament_target_dependencies(ndt_inc
    sensor_msgs
    velodyne_msgs
    pcl_conversions
)

add_library(occupancy_map occupancy_map.cpp)
target_link_libraries(occupancy_map
    glog
)
ament_target_dependencies(occupancy_map
    sensor_msgs
    velodyne_msgs
    pcl_conversions
)

add_library(map map.cpp)
target_link_libraries(map
    ndt_inc
    occupancy_map
    glog
    ${OpenCV_LIBS}
)

add_library(display display.cpp)
target_link_libraries(display
    lidar_2d_utils
    glog
)
ament_target_dependencies(display
    sensor_msgs
    velodyne_msgs
    pcl_conversions
)

add_library(eskf eskf.cpp)
add_library(ieskf ieskf.cpp)
add_library(imu_preintegration imu_preintegration.cpp)
target_link_libraries(imu_preintegration
    ${g2o_libs}
)

add_library(frontend frontend.cpp)
target_link_libraries(frontend
    display
    map
    eskf
    ieskf
    imu_preintegration
)


add_library(incrementalNDTLO incrementalNDTLO.cpp)
target_link_libraries(incrementalNDTLO
    frontend
)

add_executable(test_2d_lo test_2d_lo.cpp)
target_link_libraries(test_2d_lo
    glog
    gflags
    incrementalNDTLO
    ${libcommon}
    ${libtools}
    ${OpenCV_LIBS}
)
ament_target_dependencies(test_2d_lo
    rclcpp
    rosbag2_cpp
    sensor_msgs
    velodyne_msgs
    pcl_conversions
    livox_ros_driver2
    angles
    locsensor
    tf2_ros
    nav_msgs
    origincar_msg
)

