#include "../include/frontend.h"

#include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include <execution>

namespace sad {

bool Frontend::processScan( Scan2d::Ptr scan ){
    // 新建帧
    is_keyframe_ = false;
    current_frame_ = std::make_shared<Frame>();  
    current_frame_->id_ = frame_id_++;

    // 去畸变
    undistortAndGeneratePoints(scan);  

    // 利用scan matching来匹配地图
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
            SE2 T_wi = eskf_->getNominalPose();
            current_frame_->pose_ = T_wi;
            map_->matchScan(current_frame_);  
            eskf_->observeLidar(current_frame_->pose_);
            current_frame_->pose_ = eskf_->getNominalPose();
            convertPoints( T_wi );
        } else if ( ieskf_ ) {   // ieskf
            SE2 T_wi = ieskf_->getNominalPose();
            map_->getNdt().setSource(current_frame_);  
            ieskf_->updateUsingCustomObserve( [this]( const SE2 & init_pose, Mat8d & HT_Vinv_H, Vec8d & HT_Vinv_r ){
                map_->getNdt().computeResidualAndJacobians(init_pose, HT_Vinv_H, HT_Vinv_r );  
            });
            current_frame_->pose_ = ieskf_->getNominalPose();
            convertPoints( T_wi );
        } else {  // NDT LO
            current_frame_->pose_ = last_frame_pose_ * motion_guess_;   
            map_->matchScan(current_frame_);
        }
        motion_guess_ = last_frame_pose_.inverse() * current_frame_->pose_;  
    }

    // 执行关键帧的操作
    is_keyframe_ = isKeyFrame();
    if ( is_keyframe_ ) {
        addKeyFrame(scan);  
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
    if ( imu_states_.size() > opts_.imu_states_buffer_size_ ) imu_states_.pop_front();  
    return res;
}

bool Frontend::processOdom( const std::shared_ptr<Odom> odom ){
    if ( eskf_ ) {
        return eskf_->observeOdom(odom);
    } else if ( ieskf_ ) {
        return ieskf_->observeOdom(odom);
    }
    return false;
}

void Frontend::undistortAndGeneratePoints(Scan2d::Ptr scan){
    SE2 T_end = SE2();
    if ( eskf_ ) {
        T_end = eskf_->getNominalPose();
    } else if ( ieskf_ ) {
        T_end = ieskf_->getNominalPose();
    }
    
    bool imu_empty = ( imu_states_.empty() || ( imu_states_.size() < opts_.imu_states_buffer_size_ ) );
    
    const float time_th = 0.5;
    int scan_num = scan->ranges.size();
    double last_time = 0.0;
    if ( !imu_empty ) {  
        last_time = imu_states_.rbegin()->first;
    }

    // 提取硬编码值，方便阅读
    double angle_inc = 0.013993730768561363;
    double time_inc = 0.00025493954308331013;
    
    double scan_end_time = double(scan->header.stamp.sec) + double(scan->header.stamp.nanosec)*1e-9;
    double lidar_begin_time = scan_end_time - ( scan_num - 1 ) * time_inc;

    for ( int idx = 0; idx < scan_num; ++idx ) {
        double angle = scan->angle_min + idx * angle_inc;  
        if ( !first_scan_ && (angle < angle_min_ || angle > angle_max_) ) continue;  

        double range = scan->ranges[idx];
        if ( !first_scan_ && range > range_max_ ) continue;
        
        // 【关键修正 1】：反转时间顺序！
        // 因为 N10 驱动中 ranges[0] 是最后打出的点，它应该对应最晚的时间(scan_end_time)
        // ranges[N-1] 是最先打出的点，应该对应最早的时间(lidar_begin_time)
        double query_time = lidar_begin_time + (scan_num - 1 - idx) * time_inc;
        
        Vec2d raw_point( range * cos(angle), range * sin(angle) );
        if ( imu_empty ) {
            current_frame_->pts_.emplace_back(raw_point);
        } else {
            SE2 Ti = T_end;
            if (!poseInterp(query_time, last_time, Ti)) {
                LOG(INFO) << "interp false";
                Ti = T_end;
            }
            
            // 【关键修正 2】：修复外参坐标变换公式
            // 目标: 把 t 时刻 Lidar 系下的点 p_l_t，变换到 end 时刻 Lidar 系下的点 p_l_end
            // 推导: 
            // 1. p_i_t = T_i_l * p_l_t              (t时刻 Lidar -> t时刻 IMU)
            // 2. p_w = T_w_i_t * p_i_t               (t时刻 IMU -> World)
            // 3. p_i_end = T_w_i_end.inverse() * p_w (World -> end时刻 IMU)
            // 4. p_l_end = T_i_l.inverse() * p_i_end (end时刻 IMU -> end时刻 Lidar)
            // 合并: p_l_end = T_i_l.inverse() * T_w_i_end.inverse() * T_w_i_t * T_i_l * p_l_t
            // 错误写法: T_IL_.inverse() * Ti.inverse() * T_end * T_IL_
            // 正确写法: T_IL_.inverse() * T_end.inverse() * Ti * T_IL_
            SE2 deltaT = opts_.T_IL_.inverse() * T_end.inverse() * Ti * opts_.T_IL_;
            
            Vec2d p_compensate = deltaT * raw_point;
            current_frame_->pts_.emplace_back(p_compensate);
        }
    }
}

inline bool Frontend::poseInterp(double query_time, double last_time, SE2 & result, float time_th) {
    if (imu_states_.empty()) return false;

    if (query_time > last_time) {
        if (query_time < last_time + time_th) {
            result = imu_states_.rbegin()->second;
            return true;
        }
        return false;
    }

    if (query_time <= imu_states_.begin()->first) {
        result = imu_states_.begin()->second;
        return true;
    }

    auto iter = imu_states_.begin();
    auto next_iter = std::next(iter);
    while (next_iter != imu_states_.end() && next_iter->first < query_time) {
        ++iter;
        ++next_iter;
    }

    if (next_iter == imu_states_.end()) {
        result = iter->second;
        return true;
    }

    const double dt = next_iter->first - iter->first;
    if (dt < 1e-6) {
        result = iter->second;
        return true;
    }

    double s = (query_time - iter->first) / dt;
    s = std::clamp(s, 0.0, 1.0);   

    const SE2& T_a = iter->second;
    const SE2& T_b = next_iter->second;

    SE2 relative = T_a.inverse() * T_b;
    auto log_rel = relative.log();       
    log_rel *= s;
    SE2 delta = SE2::exp(log_rel);
    result = T_a * delta;
    return true;
}

void Frontend::convertPoints( const SE2 & T_wi ){
    SE2 deltaT = current_frame_->pose_.inverse() * T_wi; 

    #pragma omp parallel for
    for (size_t i = 0; i < current_frame_->pts_.size(); ++i ) {
        current_frame_->pts_[i] = deltaT * current_frame_->pts_[i];
    }
}

bool Frontend::isKeyFrame(){
    if (last_keyframe_ == nullptr) return true;

    delta_pose_with_kf_ = last_keyframe_->pose_.inverse() * current_frame_->pose_;
    if (delta_pose_with_kf_.translation().norm() > opts_.kf_distance_ || fabs(delta_pose_with_kf_.so2().log()) > opts_.kf_angle_rad_ ) {
        return true;
    }
    return false;
}

void Frontend::addKeyFrame(Scan2d::Ptr scan){
    current_frame_->keyframe_id_ = keyframe_id_ ++;
    map_->addKeyframe(current_frame_);
    map_->addScanInNdt(current_frame_);
    if ( opts_.localization_mode_ == false && (keyframe_id_ % (opts_.kf_add_scan_in_occu_+1)) == 0 ) {
        map_->addScanInOccupancyMap(current_frame_);  
    }
    last_keyframe_ = current_frame_;
}

} // namespace sad