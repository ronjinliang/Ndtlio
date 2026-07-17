#ifndef __INCREMENTALNDTLO_H
#define __INCREMENTALNDTLO_H

#include "../include/frame.h"
#include "../include/frontend.h"
#include "../include/map.h"
#include "../include/eskf.h"
#include "../include/ieskf.h"
#include "../common/static_imu_init.h"

namespace sad {

class IncrementalNDTLO {
public:
    IncrementalNDTLO(){}
    IncrementalNDTLO( const std::string & fileName, int with_imu = 1);
    ~IncrementalNDTLO();
    bool initIMU( IMUPtr imu );

    bool processIMU( const IMUPtr imu ) { return frontend_->processIMU(imu); }
    bool proccessOdom( const std::shared_ptr<Odom> odom ) { return frontend_->processOdom(odom); }
    bool processScan( Scan2d::Ptr scan ) { return frontend_->processScan(scan); }

    std::shared_ptr<Frontend> getFrontend() { return frontend_; }
    std::shared_ptr<ESKF> getESKF() { return eskf_; }
    std::shared_ptr<IESKF> getIESKF() { return ieskf_; }
    std::shared_ptr<Map> getMap() { return map_; }

private:
    sad::StaticIMUInit static_imu_init_;
    bool imu_init_success_ = false;

    double imu_dt_ = 0.01;
    
    Vec2d ba_ = Vec2d::Zero();
    double bg_ = 0.0;
    double gyro_var_ = 1e-2;
    double acce_var_ = 1e-2;
    double bias_gyro_var_ = 1e-4;
    double bias_acce_var_ = 1e-4;
    double odom_var_ = 0.5;

    double eskf_lidar_pos_noise_ = 0.01;
    double eskf_lidar_ang_noise_ = 1. * M_PI / 180.;
    
    int ieskf_num_iterations_ = 3; // 迭代次数
    double ieskf_eps_ = 1e-3;     // 终止迭代的dx大小
    double ieskf_info_ratio_ = 0.01;
    bool update_bias_gyro_ = true;
    bool update_bias_acce_ = true;

    double pre_info_wright_ = 0.1;
    double pre_ndt_pos_noise_ = 0.1;
    double pre_ndt_ang_noise_ = 0.1;
    bool pre_update_bias_gyro_ = true;
    bool pre_update_bias_acce_ = true;

    std::shared_ptr<Frontend> frontend_ = nullptr;
    std::shared_ptr<ESKF> eskf_ = nullptr;
    std::shared_ptr<IESKF> ieskf_ = nullptr;
    std::shared_ptr<Map> map_ = nullptr;
};

}

#endif
