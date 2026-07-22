
# IO 工具
add_library(io_utils SHARED io_utils.cpp)
target_link_libraries(io_utils
    glog
)
ament_target_dependencies(io_utils
    sensor_msgs
    locsensor
    origincar_msg
    rclcpp
    rosbag2_cpp
)