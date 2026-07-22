#ifndef __FRONTEBD_H
#define __FRONTEBD_H

#include <thread>
#include <memory>
#include <opencv2/core.hpp>

#include "2dNdtLIO/include/frame.h"
#include "2dNdtLIO/include/display.h"
#include "2dNdtLIO/include/map.h"
#include "2dNdtLIO/include/eskf.h"
#include "2dNdtLIO/include/ieskf.h"
#include "2dNdtLIO/include/loop_closure.h"

#include "2dNdtLIO/common/eigen_types.h"

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
    };

    Frontend( Options opts ) : opts_(opts) {}
    
    void setMap( std::shared_ptr<Map> map ) { map_ = map; }
    void setESKF( std::shared_ptr<ESKF> eskf ) { eskf_ = eskf; }
    void setIESKF( std::shared_ptr<IESKF> ieskf ) { ieskf_ = ieskf; }
    void setDisplay( std::shared_ptr<Display> display ) { display_ = display; }
    void setLoopClosure( std::shared_ptr<LoopClosure> loopClosure ) { loopClosure_ = loopClosure; }

    /// 单回波scan
    bool processIMU( const IMUPtr imu );
    bool processOdom( const std::shared_ptr<Odom> odom );
    bool processScan( Scan2d::Ptr scan );

    std::shared_ptr<Frame> getCurrentFrame() { return current_frame_; }

    // ROS2 节点使用
    bool isKeyframe() { return is_keyframe_; }

private:
    // 去畸变同时也转成点云
    void undistortAndGeneratePoints( Scan2d::Ptr scan );
    inline bool poseInterp(double query_time, double last_time, SE2 & result, float time_th = 0.5 );

    bool isKeyFrame();
    /// 增加一个关键帧
    void addKeyFrame(Scan2d::Ptr scan);

private:
    Options opts_;

    /// 数据成员
    size_t frame_id_ = 0;
    size_t keyframe_id_ = 0;
    bool is_keyframe_ = false;

    bool first_scan_ = true;
    SE2 last_frame_pose_ = SE2();  // 用 SE2 保存上一帧的 pose 而不是用指针访问，这是因为后端优化可能修改
    std::shared_ptr<Frame> current_frame_ = nullptr;
    std::shared_ptr<Frame> last_keyframe_ = nullptr;

    SE2 motion_guess_;  // LO 使用
    SE2 delta_pose_with_kf_;
    
    std::deque<std::pair<double, SE2>> imu_states_;

    std::shared_ptr<Map> map_ = nullptr;
    std::shared_ptr<ESKF> eskf_ = nullptr;
    std::shared_ptr<IESKF> ieskf_ = nullptr;
    std::shared_ptr<Display> display_ = nullptr;
    std::shared_ptr<LoopClosure> loopClosure_ = nullptr;

    double range_max_ = 0.0;
    double range_min_ = 0.0;
    double angle_max_ = 0.0;
    double angle_min_ = 0.0;

};

}

#endif
