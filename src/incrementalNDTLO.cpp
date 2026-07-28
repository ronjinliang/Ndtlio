#include "../include/incrementalNDTLO.h"
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace sad {

IncrementalNDTLO::IncrementalNDTLO( const std::string & fileName, int with_imu ) {
    YAML::Node config = YAML::LoadFile(fileName);

    // 前端
    Frontend::Options frontend_opts;
    frontend_opts.localization_mode_      = config["main"]["localization_mode"].as<bool>();
    frontend_opts.kf_distance_            = config["frontend"]["kf_distance"].as<double>();
    frontend_opts.kf_angle_deg_           = config["frontend"]["kf_angle_deg"].as<double>();
    frontend_opts.kf_angle_rad_           = frontend_opts.kf_angle_deg_ * M_PI / 180.;
    frontend_opts.kf_add_scan_in_occu_    = config["frontend"]["kf_add_scan_in_occu"].as<int>();
    frontend_opts.max_distance_           = config["frontend"]["max_distance"].as<double>();
    frontend_opts.angle_boarder_          = config["frontend"]["angle_boarder"].as<double>() * M_PI / 180.0;
    frontend_opts.imu_states_buffer_size_ = config["frontend"]["imu_states_buffer_size"].as<int>();
    auto t_il = config["frontend"]["T_IL"];
    double x   = t_il[0].as<double>();
    double y   = t_il[1].as<double>();
    double theta = t_il[2].as<double>();   // 单位与你的系统一致

    frontend_opts.T_IL_ = SE2(theta, Vec2d(x, y));   // 适配你实际的 SE2 实现
    LOG(INFO) << "雷达IMU外参:\n  平移(m):" << frontend_opts.T_IL_.translation().transpose() << ", 角度(rad): " << frontend_opts.T_IL_.so2().log();
    
    // 地图
    Map::Options map_opts;
    map_opts.ndt_opts_.max_iter_            = config["ndt"]["max_iter"].as<int>();
    map_opts.ndt_opts_.voxel_size_          = config["ndt"]["voxel_size"].as<double>();
    map_opts.ndt_opts_.inv_voxel_size_      = 1. / map_opts.ndt_opts_.voxel_size_;
    map_opts.ndt_opts_.min_effective_pts_   = config["ndt"]["min_effective_pts"].as<int>();
    map_opts.ndt_opts_.min_pts_in_voxel_    = config["ndt"]["min_pts_in_voxel"].as<int>();
    map_opts.ndt_opts_.max_pts_in_voxel_    = config["ndt"]["max_pts_in_voxel"].as<int>();
    map_opts.ndt_opts_.eps_                 = config["ndt"]["eps"].as<double>();
    if ( map_opts.ndt_opts_.eps_ < 0. ) map_opts.ndt_opts_.eps_ = map_opts.ndt_opts_.voxel_size_ * 1e-2;  // 1%
    map_opts.ndt_opts_.res_outlier_th_      = config["ndt"]["res_outlier_th"].as<double>();
    map_opts.ndt_opts_.capacity_            = config["ndt"]["capacity"].as<int>();
    int nearby_type                         = config["ndt"]["nearby_type"].as<int>();
    map_opts.ndt_opts_.normalizing_factor_  = config["ndt"]["normalizing_factor"].as<double>();
    map_opts.ndt_opts_.init_info_           = config["ndt"]["init_info"].as<double>();
    if      ( nearby_type == 0 ) map_opts.ndt_opts_.nearby_type_ = NdtInc2d::NearbyType::CENTER;
    else if ( nearby_type == 1 ) map_opts.ndt_opts_.nearby_type_ = NdtInc2d::NearbyType::NEARBY4;
    else if ( nearby_type == 2 ) map_opts.ndt_opts_.nearby_type_ = NdtInc2d::NearbyType::NEARBY8;
    else                         map_opts.ndt_opts_.nearby_type_ = NdtInc2d::NearbyType::NEARBY8;

    map_opts.occu_opts_.closest_th_        = config["occupancy_map"]["closest_th"].as<double>();
    map_opts.occu_opts_.endpoint_close_th_ = config["occupancy_map"]["endpoint_close_th"].as<double>();
    map_opts.occu_opts_.resolution_        = config["occupancy_map"]["resolution"].as<double>();
    map_opts.occu_opts_.inv_resolution_    = 1. / map_opts.occu_opts_.resolution_;
    map_opts.occu_opts_.image_size_        = config["occupancy_map"]["image_size"].as<int>();

    frontend_ = std::make_shared<Frontend>( std::move(frontend_opts) );
    map_ = std::make_shared<Map>( std::move(map_opts) );
    
    frontend_->setMap(map_);

    if ( with_imu == 1 ) {
        eskf_ = std::make_shared<ESKF>(ESKF::Options());
        frontend_->setESKF(eskf_);
        LOG(INFO) << "using eskf.";
    } else if ( with_imu == 2 ) {
        ieskf_ = std::make_shared<IESKF>(IESKF::Options());
        frontend_->setIESKF(ieskf_);
        LOG(INFO) << "using ieskf.";
    }

    imu_dt_                 = config["imu"]["imu_dt"].as<double>();
    ba_                     = Vec2d(config["imu"]["bax"].as<double>(), config["imu"]["bay"].as<double>());
    bg_                     = config["imu"]["bg"].as<double>();
    
    gyro_var_               = config["imu"]["gyro_var"].as<double>();
    acce_var_               = config["imu"]["acce_var"].as<double>();
    bias_gyro_var_          = config["imu"]["bias_gyro_var"].as<double>();
    bias_acce_var_          = config["imu"]["bias_acce_var"].as<double>();
    odom_var_               = config["imu"]["odom_var"].as<double>();
    update_bias_gyro_       = config["imu"]["update_bias_gyro"].as<bool>();
    update_bias_acce_       = config["imu"]["update_bias_acce"].as<bool>();

    eskf_lidar_pos_noise_   = config["imu"]["eskf"]["lidar_pos_noise"].as<double>();
    eskf_lidar_ang_noise_   = config["imu"]["eskf"]["lidar_ang_noise"].as<double>();

    ieskf_num_iterations_   = config["imu"]["ieskf"]["num_iterations"].as<int>();
    ieskf_eps_              = config["imu"]["ieskf"]["eps"].as<double>();
    ieskf_info_ratio_       = config["imu"]["ieskf"]["info_ratio"].as<double>();
}

IncrementalNDTLO::~IncrementalNDTLO(){
}

bool IncrementalNDTLO::initIMU( IMUPtr imu ){
    // if ( !static_imu_init_.InitSuccess() && !imu_init_success_ ) {  // TUDO 自己的数据集暂时先注释掉这个
    //     static_imu_init_.AddIMU(*imu);
    //     return false;
    // }
    // if ( !imu_init_success_ ) {
        imu_init_success_ = true;
        
        double gyro_var      = gyro_var_;
        double acce_var      = acce_var_;
        double bias_gyro_var = bias_gyro_var_;
        double bias_acce_var = bias_acce_var_;
        if ( gyro_var < 0 ) gyro_var = sqrt(static_imu_init_.GetCovGyro()(2));
        if ( acce_var < 0 ) acce_var = sqrt(static_imu_init_.GetCovAcce()(0));
        // 通常偏差游走是测量噪声的1/10到1/100
        if ( bias_gyro_var < 0 ) bias_gyro_var = 0.01 * gyro_var;
        if ( bias_acce_var < 0 ) bias_acce_var = 0.01 * acce_var;
        // double bg = static_imu_init_.GetInitBg()(2);
        // Vec2d ba = Vec2d(
        //     static_imu_init_.GetInitBa()(0) + static_imu_init_.GetGravity()(0),
        //     static_imu_init_.GetInitBa()(1) + static_imu_init_.GetGravity()(1));
        
        // 自己的数据集直接设定  origincar3 数据集没有停10s
        Vec2d ba = ba_;
        double bg = bg_;

        LOG(INFO) << "gyro var: " << gyro_var << ", acce var: " << acce_var 
                << ", bias gyro var: " << bias_gyro_var << ", bias acce var: " << bias_acce_var
                << ", bg: " << bg << ", ba: " << ba.transpose();

        if ( eskf_ ) {
            ESKF::Options opts;
            opts.imu_dt_          = imu_dt_;
            opts.gyro_var_        = gyro_var;
            opts.acce_var_        = acce_var;
            opts.bias_gyro_var_   = bias_gyro_var;
            opts.bias_acce_var_   = bias_acce_var;
            opts.odom_var_        = odom_var_;
            opts.lidar_pos_noise_ = eskf_lidar_pos_noise_;
            opts.lidar_ang_noise_ = eskf_lidar_ang_noise_;
            opts.update_bias_gyro_ = update_bias_gyro_;
            opts.update_bias_acce_ = update_bias_acce_;
            eskf_->setInitCondition(opts, bg, ba);
            LOG(INFO) << "ESKF: imu init finished.";
        } else if ( ieskf_ ) {
            IESKF::Options opts;
            opts.imu_dt_         = imu_dt_;
            opts.num_iterations_ = ieskf_num_iterations_;  // 迭代次数
            opts.eps_            = ieskf_eps_;             // 终止迭代的dx大小
            opts.info_ratio_     = ieskf_info_ratio_;

            opts.gyro_var_       = gyro_var;
            opts.acce_var_       = acce_var;
            opts.bias_gyro_var_  = bias_gyro_var;
            opts.bias_acce_var_  = bias_acce_var;

            opts.odom_var_         = odom_var_;
            opts.update_bias_gyro_ = update_bias_gyro_;
            opts.update_bias_acce_ = update_bias_acce_;

            ieskf_->setInitCondition(opts, bg, ba);
            LOG(INFO) << "IESKF: imu init finished.";
        }
        return frontend_->processIMU(imu);
    // } else {
    //     return frontend_->processIMU(imu);
    // }
}



}
