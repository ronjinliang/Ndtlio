#include "2dNdtLIO(preintegration)/map.h"
#include <yaml-cpp/yaml.h>

namespace sad {

/// 在栅格地图中增加一个帧，前端往这加的是关键帧
void Map::addScanInOccupancyMap(std::shared_ptr<Frame> frame, Scan2d::Ptr scan){
    occu_map_.addLidarFrame(frame, scan);
}

void Map::addScanInNdt( std::shared_ptr<Frame> frame ){
    ndt_.addScan( frame );
}

/// 将frame与本submap进行匹配，计算frame->pose
bool Map::matchScan(std::shared_ptr<Frame> frame){
    /// 当前帧与子地图匹配
    ndt_.setSource( frame );       // 当前帧
    ndt_.alignNdt( frame->pose_ ); // 当前帧的 pose
    return true;
}

void Map::loadParamsFromYAML( const std::string & fileName ){
    YAML::Node config = YAML::LoadFile(fileName);
    NdtInc2d::Options ndt_opts;
    ndt_opts.max_iter_            = config["ndt"]["max_iter"].as<int>();
    ndt_opts.voxel_size_          = config["ndt"]["voxel_size"].as<double>();
    ndt_opts.inv_voxel_size_      = 1. / ndt_opts.voxel_size_;
    ndt_opts.min_effective_pts_   = config["ndt"]["min_effective_pts"].as<int>();
    ndt_opts.min_pts_in_voxel_    = config["ndt"]["min_pts_in_voxel"].as<int>();
    ndt_opts.max_pts_in_voxel_    = config["ndt"]["max_pts_in_voxel"].as<int>();

    ndt_opts.eps_                 = config["ndt"]["eps"].as<double>();
    if ( ndt_opts.eps_ < 0. ) ndt_opts.eps_ = ndt_opts.voxel_size_ * 1e-2;  // 1%

    ndt_opts.res_outlier_th_      = config["ndt"]["res_outlier_th"].as<double>();
    ndt_opts.capacity_            = config["ndt"]["capacity"].as<int>();
    int nearby_type               = config["ndt"]["nearby_type"].as<int>();
    ndt_opts.normalizing_factor_  = config["ndt"]["normalizing_factor"].as<double>();
    ndt_opts.init_info_           = config["ndt"]["init_info"].as<double>();

    if      ( nearby_type == 0 ) ndt_opts.nearby_type_ = NdtInc2d::NearbyType::CENTER;
    else if ( nearby_type == 1 ) ndt_opts.nearby_type_ = NdtInc2d::NearbyType::NEARBY4;
    else if ( nearby_type == 2 ) ndt_opts.nearby_type_ = NdtInc2d::NearbyType::NEARBY8;
    else                         ndt_opts.nearby_type_ = NdtInc2d::NearbyType::NEARBY8;
    ndt_.loadParams(ndt_opts);

    OccupancyMap::Options occu_opts;
    occu_opts.closest_th_        = config["occupancy_map"]["closest_th"].as<double>();
    occu_opts.endpoint_close_th_ = config["occupancy_map"]["endpoint_close_th"].as<double>();
    occu_opts.resolution_        = config["occupancy_map"]["resolution"].as<double>();
    occu_opts.inv_resolution_    = 1./occu_opts.resolution_;
    occu_opts.image_size_        = config["occupancy_map"]["image_size"].as<int>();
    occu_opts.model_size_        = config["occupancy_map"]["model_size"].as<int>();
    int method                   = config["occupancy_map"]["method"].as<int>();
    if ( method == 0 ) {         // 直接栅格化算法
        occu_opts.method_ = OccupancyMap::GridMethod::BRESENHAM;
    } else if ( method == 1 ) {  // 模板化算法
        occu_opts.method_ = OccupancyMap::GridMethod::MODEL_POINTS;
    } else {  // 默认
        occu_opts.method_ = OccupancyMap::GridMethod::BRESENHAM;
    }
    occu_map_.loadParams(occu_opts);
}

}
