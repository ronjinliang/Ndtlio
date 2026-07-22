#ifndef __INCREMENTALNDTLO_H
#define __INCREMENTALNDTLO_H

#include "2dNdtLIO(preintegration)/frontend.h"
#include "2dNdtLIO(preintegration)/map.h"
#include "2dNdtLIO(preintegration)/display.h"
#include "2dNdtLIO(preintegration)/eskf.h"
#include "2dNdtLIO(preintegration)/ieskf.h"
#include "2dNdtLIO(preintegration)/imu_preintegration.h"
#include "ch3/static_imu_init.h"

namespace sad {

class IncrementalNDTLO {
public:
    IncrementalNDTLO( const std::string & fileName, int with_imu = 3 );
    bool initIMU( IMUPtr imu );

    bool processIMU( const IMUPtr imu ) { return frontend_->processIMU(imu); }
    bool proccessOdom( const std::shared_ptr<Odom> odom ) { return frontend_->processOdom(odom); }
    bool processScan( Scan2d::Ptr scan ) { return frontend_->processScan(scan); }
    bool provessMultiScan( MultiScan2d::SharedPtr scan ) { return frontend_->processScan(MultiToScan2d(scan)); }

    void close();

    const cv::Mat getGlobalMap( int size = 2000 ){ return display_->getGlobalMap( size ); }

private:
    sad::StaticIMUInit static_imu_init_;
    bool imu_init_success_ = false;

    double imu_dt_ = 0.01;
    
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
    bool ieskf_update_bias_gyro_ = true;
    bool ieskf_update_bias_acce_ = true;

    double pre_info_wright_ = 0.1;
    double pre_ndt_pos_noise_ = 0.1;
    double pre_ndt_ang_noise_ = 0.1;
    bool pre_update_bias_gyro_ = true;
    bool pre_update_bias_acce_ = true;

    std::shared_ptr<Frontend> frontend_ = nullptr;
    std::shared_ptr<ESKF> eskf_ = nullptr;
    std::shared_ptr<IESKF> ieskf_ = nullptr;
    std::shared_ptr<IMUPreintegration> preintegration_ = nullptr;
    // std::shared_ptr<LoopClosing> loop_closing_ = nullptr;  // 回环检测
    std::shared_ptr<Map> map_ = nullptr;
    std::shared_ptr<Display> display_ = nullptr;
};

}

#endif
