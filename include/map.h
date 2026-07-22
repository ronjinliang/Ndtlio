#ifndef __MAP_H
#define __MAP_H

#include "../include/frame.h"
#include "../include/ndt_inc.h"
#include "../include/occupancy_map.h"
#include <opencv2/opencv.hpp>
#include <map>
#include <thread>
#include <atomic>

namespace sad {

class Map {
public:
    struct Options {
        NdtInc2d::Options ndt_opts_;
        OccupancyMap::Options occu_opts_;
    };

    Map( Options opts );

    NdtInc2d & getNdt();
    NdtInc2d::Options getNdtOptions();  // 回环检测使用, 重新创建 ndt
    OccupancyMap & getOccupancyMap();
    
    std::map<size_t, std::shared_ptr<Frame>> & getAllFrames();
    std::map<size_t, std::shared_ptr<Frame>> getAllFramesByCopy();
    size_t getFramesNum();

    /// 在 NDT 中增加一个帧 与 resetNdt 互斥
    void addScanInNdt( std::shared_ptr<Frame> frame );
    /// 在栅格地图中增加一个帧，前端往这加的是关键帧
    void addScanInOccupancyMap(std::shared_ptr<Frame> frame);
    /// 添加关键帧
    void addKeyframe( std::shared_ptr<Frame> frame );

    std::shared_ptr<Frame> getKf( size_t & id );
    void updateKf( size_t & id, SE2 pose );

    bool matchScan( std::shared_ptr<Frame> frame );
    
private:
    std::mutex data_mutex_;

    Options opts_;

    std::map<size_t, std::shared_ptr<Frame>> frames_;  // 所有关键帧
    NdtInc2d ndt_;          // 用于匹配
    OccupancyMap occu_map_; // 用于生成栅格地图
};


}


#endif
