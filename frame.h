#ifndef __FRAME_H
#define __FRAME_H

#include "common/eigen_types.h"
#include "common/lidar_utils.h"

#include <glog/logging.h>
#include <fstream>

namespace sad {

/**
 * 一次2d scan
 */
struct Frame {
    Frame() {}
    // Frame(Scan2d::Ptr scan) : scan_(scan) {}

    // /// 将当前帧存入文本文件以供离线调用
    // void Dump(const std::string & filename){
    //     std::ofstream fout(filename);
    //     fout << id_ << " " << keyframe_id_ << " " << timestamp_ << std::endl;
    //     fout << pose_.translation()[0] << " " << pose_.translation()[1] << " " << pose_.so2().log() << std::endl;
    //     fout << scan_->angle_min << " " << scan_->angle_max << " " << scan_->angle_increment << " " << scan_->range_min
    //         << " " << scan_->range_max << " " << scan_->ranges.size() << std::endl;
    //     for (auto& r : scan_->ranges) {
    //         fout << r << " ";
    //     }
    //     fout.close();
    // }

    // /// 从文件读取frame数据
    // void Load(const std::string & filename) {
    //     std::ifstream fin(filename);
    //     if (!fin) {
    //         LOG(ERROR) << "cannot load from " << filename;
    //         return;
    //     }

    //     fin >> id_ >> keyframe_id_ >> timestamp_ >> pose_.translation()[0] >> pose_.translation()[1];
    //     double theta = 0;
    //     fin >> theta;
    //     pose_.so2() = SO2::exp(theta);
    //     scan_.reset(new Scan2d);
    //     fin >> scan_->angle_min >> scan_->angle_max >> scan_->angle_increment >> scan_->range_min >> scan_->range_max;

    //     int range_size;
    //     fin >> range_size;
    //     for (int i = 0; i < range_size; ++i) {
    //         double r;
    //         fin >> r;
    //         scan_->ranges.emplace_back(r);
    //     }
    // }

    size_t id_ = 0;               // scan id
    size_t keyframe_id_ = 0;      // 关键帧 id
    double timestamp_ = 0;        // 时间戳，一般不用
    // Scan2d::Ptr scan_ = nullptr;  // 激光扫描数据
    SE2 pose_;                    // 位姿，scan to world, T_w_c
    std::vector<Vec2d> pts_;
};
}

#endif

