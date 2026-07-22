#ifndef __IO_UTILS_H
#define __IO_UTILS_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <locsensor/msg/ivsensorimu.hpp>
#include <origincar_msg/msg/data.hpp>

#include <fstream>
#include <functional>

#include "2dNdtLIO/include/frame.h"
#include "2dNdtLIO/common/imu.h"
#include "2dNdtLIO/common/math_utils.h"
#include "2dNdtLIO/common/odom.h"

namespace sad {

/**
 * ROSBAG IO
 * 指定一个包名，添加一些回调函数，就可以顺序遍历这个包
 */
class RosbagIO {
   public:
    explicit RosbagIO(std::string bag_file) : bag_file_(std::move(bag_file)) {}

    using MessageProcessFunction = std::function<bool(rosbag2_storage::SerializedBagMessageSharedPtr msg)>;

    /// 一些方便直接使用的topics, messages
    using Scan2DHandle = std::function<bool(sensor_msgs::msg::LaserScan::SharedPtr)>;
    using OdomHandle = std::function<bool(std::shared_ptr<Odom>)>;
    using ImuHandle = std::function<bool(IMUPtr)>;

    // 遍历文件内容，调用回调函数
    void Go();

    /// 通用处理函数
    RosbagIO &AddHandle(const std::string &topic_name, MessageProcessFunction func) {
        process_func_.emplace(topic_name, func);
        return *this;
    }

    /// 2D激光处理
    RosbagIO &AddScan2DHandle( const std::string &topic_name, Scan2DHandle f);
    RosbagIO &AddMultiScanHandle( const std::string &topic_name, Scan2DHandle f);
    RosbagIO &AddOdomHandle( const std::string & topic_name, OdomHandle f );
    RosbagIO &AddImuHandle( const std::string &topic_name, ImuHandle f);   // 传入话题名来解析

    /// 清除现有的处理函数
    void CleanProcessFunc() { process_func_.clear(); }

private:

    inline Scan2d::SharedPtr MultiToScan2d(sensor_msgs::msg::MultiEchoLaserScan::SharedPtr mscan) {
        Scan2d::SharedPtr scan(new Scan2d);
        scan->header = mscan->header;
        scan->range_max = mscan->range_max;
        scan->range_min = mscan->range_min;
        scan->angle_increment = mscan->angle_increment;
        scan->angle_max = mscan->angle_max;
        scan->angle_min = mscan->angle_min;
        for (auto r : mscan->ranges) {
            if (r.echoes.empty()) {
                scan->ranges.emplace_back(scan->range_max + 1.0);
            } else {
                scan->ranges.emplace_back(r.echoes[0]);
            }
        }
        for (auto i : mscan->intensities) {
            if (i.echoes.empty()) {
                scan->intensities.emplace_back(0);
            } else {
                scan->intensities.emplace_back(i.echoes[0]);
            }
        }
        scan->scan_time = mscan->scan_time;
        scan->time_increment = mscan->time_increment;
        // limit range max
        // scan->range_max = 20.0;
        return scan;
    }

private:

    std::map<std::string, MessageProcessFunction> process_func_;
    std::string bag_file_;

};

}  // namespace sad


#endif
