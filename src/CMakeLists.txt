add_library( NDT_LIO STATIC
    ndt_inc.cpp
    occupancy_map.cpp
    map.cpp
    eskf.cpp
    ieskf.cpp
    frontend.cpp
    incrementalNDTLO.cpp
)
target_link_libraries( NDT_LIO
    TBB::tbb
    glog
    ${OpenCV_LIBS}
)
ament_target_dependencies( NDT_LIO
    sensor_msgs
)