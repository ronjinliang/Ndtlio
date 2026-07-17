#ifndef __FRAME_H
#define __FRAME_H

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/header.hpp>

#include "../common/eigen_types.h"

#include <glog/logging.h>
#include <fstream>

using Scan2d = sensor_msgs::msg::LaserScan;

namespace sad {

/**
 * 一次2d scan
 */
struct Frame {
    Frame() {}

    size_t id_ = 0;               // scan id
    size_t keyframe_id_ = 0;      // 关键帧 id
    double timestamp_ = 0;        // 时间戳，一般不用
    SE2 pose_;                    // 位姿，scan to world, T_w_l
    std::vector<Vec2d> pts_;
};
}

#endif

