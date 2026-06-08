#include "NdtLIO/include/frontend.h"

#include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include <execution>

namespace sad {

bool Frontend::processScan( Scan2d::Ptr scan ){
    // 新建帧
    is_keyframe_ = false;
    current_frame_ = std::make_shared<Frame>();  // 新的 frame，暂时还未传给 loop_closing 和 display
    current_frame_->id_ = frame_id_++;

    // 去畸变
    undistortAndGeneratePoints(scan);  // 去畸变同时转成点云

    // 利用scan matching来匹配地图
    // 第一帧无法匹配，且是 keyframe，直接加入到occupancy map
    if ( first_scan_ ) {
        first_scan_ = false;
        range_max_  = scan->range_max > opts_.max_distance_ ? opts_.max_distance_ : scan->range_max;
        range_min_  = scan->range_min;
        angle_max_  = scan->angle_max - opts_.angle_boarder_;
        angle_min_  = scan->angle_min + opts_.angle_boarder_;
        LOG(INFO) << "range max: " << range_max_              << ", range min: " << range_min_
                << ", angle max: " << angle_max_ * 180.0/M_PI << ", angle min: " << angle_min_ * 180.0/M_PI;
    } else {
        
        if ( eskf_ ) {  // eskf
            // ESKF 先验(初值)
            current_frame_->pose_ = eskf_->getNominalPose();
            map_->matchScan(current_frame_);  // 先匹配后融合   雷达坐标系
            eskf_->observeLidar(current_frame_->pose_);
            current_frame_->pose_ = eskf_->getNominalPose();
        } else if ( ieskf_ ) {   // ieskf
            map_->getNdt().setSource(current_frame_);  // ieskf 要单独设置一下
            ieskf_->updateUsingCustomObserve( [this]( const SE2 & init_pose, Mat8d & HT_Vinv_H, Vec8d & HT_Vinv_r ){
                map_->getNdt().conputeResidualAndJacobians(init_pose, HT_Vinv_H, HT_Vinv_r );  // 先计算矩阵和误差, 后融合
            });
            current_frame_->pose_ = ieskf_->getNominalPose();
        } else {  // NDT LO
            // set pose from last frame
            current_frame_->pose_ = last_frame_pose_ * motion_guess_;   // T_wl1 * T_l1l2
            map_->matchScan(current_frame_);
        }
        motion_guess_ = last_frame_pose_.inverse() * current_frame_->pose_;  // T_l1w * T_wl2 = T_l1l2
    }

    // 执行关键帧的操作
    is_keyframe_ = isKeyFrame();
    if ( is_keyframe_ ) {
        addKeyFrame(scan);  // scan 放入 occupancy map 和 ndt
    }

    last_frame_pose_ = current_frame_->pose_;
    return true;
}

bool Frontend::processIMU( const IMUPtr imu ){
    bool res = false;
    if ( eskf_ ) {
        res = eskf_->predict(imu);
        imu_states_.push_back({imu->timestamp_, eskf_->getNominalPose()} );
    } else if ( ieskf_ ) {
        res = ieskf_->predict(imu);
        imu_states_.push_back({imu->timestamp_, ieskf_->getNominalPose()} );
    }
    if ( imu_states_.size() == opts_.imu_states_buffer_size_ ) imu_states_.pop_front();  // 维持10个imu数据
    return res;
}

bool Frontend::processOdom( const std::shared_ptr<Odom> odom ){
    if ( eskf_ ) {
        return eskf_->observeOdom(odom);
    } else if ( ieskf_ ) {
        return ieskf_->observeOdom(odom);
    }
}

void Frontend::undistortAndGeneratePoints(Scan2d::Ptr scan){
    SE2 T_end = SE2();
    if ( eskf_ ) {
        T_end = eskf_->getNominalPose();
    } else if ( ieskf_ ) {
        T_end = ieskf_->getNominalPose();
    }
    bool imu_empty = imu_states_.empty();
    
    const float time_th = 0.5;
    int scan_num = scan->ranges.size();
    double lidar_begin_time = 0.0;
    double last_time = 0.0;
    if ( !imu_empty ) {  // 直接 LO
        lidar_begin_time = double(scan->header.stamp.sec) + double(scan->header.stamp.nanosec)*1e-9 - scan->time_increment * scan_num;
        last_time = imu_states_.rbegin()->first;
    }

    // 不能用并发, 并发需要提前分配空间, 但是在过程中需要滤波
    for ( int idx = 0; idx < scan_num; ++idx ) {
        double angle = scan->angle_min + idx * scan->angle_increment;  // 这个跟 range 对应的
        if ( !first_scan_ && (angle < angle_min_ || angle > angle_max_) ) continue;  // sb 玩意第一帧空的

        // if ( !first_scan_ && ( angle > 5./6.*M_PI && angle < 7./6.*M_PI) ) continue;   // 自己数据集去掉背后的数据  TUDO

        double range = scan->ranges[idx];
        if ( !first_scan_ && (range < range_min_ || range > range_max_) ) continue;
        double query_time = lidar_begin_time + idx * scan->time_increment;
        Vec2d raw_point( range * cos(angle), range * sin(angle) );
        if ( imu_empty ) {
            current_frame_->pts_.emplace_back(raw_point);
        } else {
            SE2 Ti = T_end;
            if (!poseInterp(query_time, last_time, Ti)) {
                LOG(INFO) << "interp false";
                Ti = T_end;
            }
            SE2 deltaT = Ti.inverse() * T_end;
            // T_end:T_w_i
            // Ti_t: T_w_i_t  t时刻 变换到 t 时刻
            Vec2d p_compensate = deltaT * raw_point;
            current_frame_->pts_.emplace_back(p_compensate);
        }
        
    }
}

inline bool Frontend::poseInterp(double query_time, double last_time, SE2 & result, float time_th ){
    if ( query_time > last_time ) {
        if (query_time < (last_time + time_th)) {
            // 尚可接受
            result = imu_states_.rbegin()->second;
            return true;
        }
        return false;
    }

    auto match_iter = imu_states_.begin();
    for (auto iter = imu_states_.begin(); iter != imu_states_.end(); ++iter) {
        auto next_iter = iter;
        ++next_iter;
        if ( iter->first < query_time && next_iter->first >= query_time) {
            match_iter = iter;
            break;
        }
    }

    auto match_iter_n = match_iter;
    ++match_iter_n;

    double dt = match_iter_n->first - match_iter->first;
    double s = (query_time - match_iter->first) / dt;   // s=0 时为第一帧，s=1时为next
    // 出现了 dt为0的bug
    if (fabs(dt) < 1e-6) {
        result = match_iter->second;
        return true;
    }
    SE2 pose_first = match_iter->second;
    SE2 pose_next = match_iter_n->second;
    // 角度需要考虑周期
    double theta_first = pose_first.so2().log();
    double theta_next  = pose_next.so2().log();
    double delta_angle = theta_next - theta_first;
    if ( delta_angle > M_PI)   delta_angle -= 2*M_PI;
    if ( delta_angle < -M_PI ) delta_angle += 2*M_PI;
    // 平移插值
    double interp_angle = theta_first + s * delta_angle;

    Vec2d interp_t = pose_first.translation() * (1-s) + pose_next.translation() * s;
    result = SE2( interp_angle, interp_t );
    return true;
}

/// 判定当前帧是否为关键帧
bool Frontend::isKeyFrame(){
    // 刚启动，没有last frame，第一帧就是 keyframe
    if (last_keyframe_ == nullptr) return true;

    // curr_pose = last_pose * delta
    // T_l1_l2 = T_l1_w * T_w_l2
    delta_pose_with_kf_ = last_keyframe_->pose_.inverse() * current_frame_->pose_;
    if (delta_pose_with_kf_.translation().norm() > opts_.kf_distance_ || fabs(delta_pose_with_kf_.so2().log()) > opts_.kf_angle_rad_ ) {
        return true;
    }
    return false;
}


/// 增加一个关键帧
void Frontend::addKeyFrame(Scan2d::Ptr scan){
    current_frame_->keyframe_id_ = keyframe_id_ ++;
    map_->addKeyframe(current_frame_);
    map_->addScanInNdt(current_frame_);
    // 定位模式不用构建占据图
    if ( opts_.localization_mode_ == false && (keyframe_id_ % (opts_.kf_add_scan_in_occu_+1)) == 0 ) {
        map_->addScanInOccupancyMap(current_frame_);  // 每x关键帧放一次
    }
    last_keyframe_ = current_frame_;
}




}


