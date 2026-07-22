#include "../include/map.h"
#include <yaml-cpp/yaml.h>

namespace sad {

Map::Map( Options opts ) : opts_(opts) {
    ndt_.loadParams(opts_.ndt_opts_);
    occu_map_.loadParams(opts_.occu_opts_);
}

NdtInc2d & Map::getNdt() { return ndt_; }
NdtInc2d::Options Map::getNdtOptions() { return opts_.ndt_opts_; }  // 回环检测使用, 重新创建 ndt
OccupancyMap & Map::getOccupancyMap() { return occu_map_; }

std::map<size_t, std::shared_ptr<Frame>> & Map::getAllFrames() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return frames_;
}

// 回环检测使用
std::map<size_t, std::shared_ptr<Frame>> Map::getAllFramesByCopy() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return frames_;
}

size_t Map::getFramesNum() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return frames_.size();
}

/// 在 NDT 中增加一个帧 与 resetNdt 互斥
void Map::addScanInNdt( std::shared_ptr<Frame> frame ){
    ndt_.addScan( frame );
}

/// 在栅格地图中增加一个帧，前端往这加的是关键帧
void Map::addScanInOccupancyMap(std::shared_ptr<Frame> frame){
    occu_map_.addLidarFrame(frame);
}

/// 添加关键帧
void Map::addKeyframe( std::shared_ptr<Frame> frame ) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    frames_.emplace( frame->keyframe_id_ , frame);
}

std::shared_ptr<Frame> Map::getKf( size_t & id ) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    // if ( id >= frames_.size() ) return nullptr;
    return frames_[id];
}

void Map::updateKf( size_t & id, SE2 pose ) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    frames_[id]->pose_ = std::move( pose );
}

/// 将frame与本submap进行匹配，计算frame->pose
bool Map::matchScan(std::shared_ptr<Frame> frame){
    /// 当前帧与子地图匹配
    ndt_.setSource( frame );       // 当前帧
    ndt_.alignNdt( frame->pose_ ); // 当前帧的 pose
    return true;
}



}
