#ifndef __FRONTEBD_H
#define __FRONTEBD_H

#include <thread>
#include <memory>
#include <opencv2/core.hpp>

#include "2dNdtLIO(preintegration)/frame.h"
#include "2dNdtLIO(preintegration)/display.h"
#include "2dNdtLIO(preintegration)/map.h"
#include "2dNdtLIO(preintegration)/eskf.h"
#include "2dNdtLIO(preintegration)/ieskf.h"
#include "2dNdtLIO(preintegration)/imu_preintegration.h"

#include "common/eigen_types.h"
#include "common/lidar_utils.h"

namespace sad {

class Frontend {
public:
    struct Options {
        double kf_distance_ = 0.1;  // 关键帧距离
        double kf_angle_deg_ = 15;  // 关键帧角度
        double kf_angle_rad_ = kf_angle_deg_ * M_PI / 180.;  // 关键帧角度
        int kf_add_scan_in_occu_ = 10;
        double max_distance_ = 20.0;              // 最远距离 滤掉部分scan
        double angle_boarder_ = 30.0;
        int imu_states_buffer_size_ = 20;
        bool display_ = true;
    };

    Frontend( const std::string & fileName );
    
    void setMap( std::shared_ptr<Map> map ) { map_ = map; }
    void setESKF( std::shared_ptr<ESKF> eskf ) { eskf_ = eskf; }
    void setIESKF( std::shared_ptr<IESKF> ieskf ) { ieskf_ = ieskf; }
    void setIMUPreintegration( std::shared_ptr<IMUPreintegration> preintegration ) { preintegration_ = preintegration; }
    void setDisplay( std::shared_ptr<Display> display ) { display_ = display; }

    /// 单回波scan
    bool processIMU( const IMUPtr imu );
    bool processOdom( const std::shared_ptr<Odom> odom );
    bool processScan( Scan2d::Ptr scan );

private:
    // 去畸变同时也转成点云
    void undistortAndGeneratePoints( Scan2d::Ptr scan );
    inline bool poseInterp(double query_time, double last_time, SE2 & result, float time_th = 0.5 );

    bool isKeyFrame();
    /// 增加一个关键帧
    void addKeyFrame(Scan2d::Ptr scan);

    void loadParamsFromYAML( const std::string & fileName );

private:
    Options opts_;

    /// 数据成员
    size_t frame_id_ = 0;
    size_t keyframe_id_ = 0;

    bool first_scan_ = true;
    std::shared_ptr<Frame> current_frame_ = nullptr;
    std::shared_ptr<Frame> last_frame_ = nullptr;
    std::shared_ptr<Frame> last_keyframe_ = nullptr;

    SE2 motion_guess_;
    
    std::deque<std::pair<double, SE2>> imu_states_;

    std::shared_ptr<Map> map_ = nullptr;
    std::shared_ptr<ESKF> eskf_ = nullptr;
    std::shared_ptr<IESKF> ieskf_ = nullptr;
    std::shared_ptr<IMUPreintegration> preintegration_ = nullptr;
    std::shared_ptr<Display> display_ = nullptr;

    double range_max_ = 0.0;
    double range_min_ = 0.0;
    double angle_max_ = 0.0;
    double angle_min_ = 0.0;

};

}

#endif
