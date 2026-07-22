#ifndef __MAP_H
#define __MAP_H

#include "2dNdtLIO(preintegration)/frame.h"
#include "2dNdtLIO(preintegration)/ndt_inc.h"
#include "2dNdtLIO(preintegration)/occupancy_map.h"
#include <opencv2/opencv.hpp>
#include <map>
#include <thread>

namespace sad {
class Map {
public:
    Map( const std::string & fileName ) {
        loadParamsFromYAML(fileName);
    }

    NdtInc2d & getNdt() { return ndt_; }
    OccupancyMap & getOccupancyMap() { return occu_map_; }
    std::vector<std::shared_ptr<Frame>> & getAllFrames() { return frames_; }
    size_t getFramesNum() { return frames_.size() ;}

    void addKeyframe( std::shared_ptr<Frame> frame ) {
        std::unique_lock<std::mutex> lock(data_mutex_);
        frames_.emplace_back(frame);
    }
    /// 在栅格地图中增加一个帧，前端往这加的是关键帧
    void addScanInOccupancyMap(std::shared_ptr<Frame> frame, Scan2d::Ptr scan);
    /// 在 NDT 中增加一个帧
    void addScanInNdt( std::shared_ptr<Frame> frame );

    bool matchScan(std::shared_ptr<Frame> frame);

private:

    void loadParamsFromYAML( const std::string & fileName );

private:
    std::mutex data_mutex_;

    std::vector<std::shared_ptr<Frame>> frames_; // 所有关键帧
    NdtInc2d ndt_;                               // 用于匹配
    OccupancyMap occu_map_;                      // 用于生成栅格地图
};


}


#endif
