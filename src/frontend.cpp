#include "../include/frontend.h"

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
        // 因为假设IMU与雷达位置重叠，所以 T_wi 等价于 T_wl
        if ( eskf_ ) {  // eskf
            // ESKF 先验(初值)
            SE2 T_wi = eskf_->getNominalPose();
            current_frame_->pose_ = T_wi;
            map_->matchScan(current_frame_);  // 先匹配后融合   雷达坐标系
            eskf_->observeLidar(current_frame_->pose_);
            current_frame_->pose_ = eskf_->getNominalPose();
            convertPoints( T_wi );
        } else if ( ieskf_ ) {   // ieskf
            SE2 T_wi = ieskf_->getNominalPose();
            map_->getNdt().setSource(current_frame_);  // ieskf 要单独设置一下
            ieskf_->updateUsingCustomObserve( [this]( const SE2 & init_pose, Mat8d & HT_Vinv_H, Vec8d & HT_Vinv_r ){
                map_->getNdt().computeResidualAndJacobians(init_pose, HT_Vinv_H, HT_Vinv_r );  // 先计算矩阵和误差, 后融合
            });
            current_frame_->pose_ = ieskf_->getNominalPose();
            convertPoints( T_wi );
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
    return false;
}

void Frontend::undistortAndGeneratePoints(Scan2d::Ptr scan){
    SE2 T_end = SE2();
    double lidar_begin_time = double(scan->header.stamp.sec) + double(scan->header.stamp.nanosec)*1e-9;
    if ( eskf_ ) {
        T_end = eskf_->getNominalPose();
    } else if ( ieskf_ ) {
        T_end = ieskf_->getNominalPose();
    }
    
    bool imu_empty = imu_states_.empty();
    
    const float time_th = 0.5;
    int scan_num = scan->ranges.size();
    double lidar_end_time = 0.0;
    double last_time = 0.0;
    if ( !imu_empty ) {  // 直接 LO
        lidar_end_time = lidar_begin_time + scan->time_increment * scan_num;
        last_time = imu_states_.rbegin()->first;
    }

    // 不能用并发, 并发需要提前分配空间, 但是在过程中需要滤波
    for ( int idx = 0; idx < scan_num; ++idx ) {
        double angle = scan->angle_min + idx * scan->angle_increment;  // 这个跟 range 对应的
        if ( !first_scan_ && (angle < angle_min_ || angle > angle_max_) ) continue;  // sb 玩意第一帧空的

        // if ( !first_scan_ && ( angle > 5./6.*M_PI && angle < 7./6.*M_PI) ) continue;   // 自己数据集去掉背后的数据  TUDO

        double range = scan->ranges[idx];
        // if ( !first_scan_ && (range < range_min_ || range > range_max_) ) continue;
        if ( !first_scan_ && range > range_max_ ) continue;
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
            // SE2 deltaT = Ti.inverse() * T_end;
            SE2 deltaT = T_end.inverse() * Ti;
            // T_end:T_w_i_t
            // Ti_t: T_w_i_i  i 时刻变换到 t 时刻
            Vec2d p_compensate = deltaT * raw_point;
            current_frame_->pts_.emplace_back(p_compensate);
        }
        
    }
}

inline bool Frontend::poseInterp(double query_time, double last_time, SE2 & result, float time_th) {
    if (imu_states_.empty()) return false;

    // 1. 查询时间晚于最新状态的处理（短暂外推）
    if (query_time > last_time) {
        if (query_time < last_time + time_th) {
            result = imu_states_.rbegin()->second;
            return true;
        }
        return false;
    }

    // 2. 边界保护：早于或等于最早状态
    if (query_time <= imu_states_.begin()->first) {
        result = imu_states_.begin()->second;
        return true;
    }

    // 3. 定位到包含 query_time 的区间 [iter->first, next_iter->first]
    auto iter = imu_states_.begin();
    auto next_iter = std::next(iter);
    while (next_iter != imu_states_.end() && next_iter->first < query_time) {
        ++iter;
        ++next_iter;
    }

    // 安全保护：若未找到（理论上不会），回退到最近的状态
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
    s = std::clamp(s, 0.0, 1.0);   // 防止数值微小越界

    const SE2& T_a = iter->second;
    const SE2& T_b = next_iter->second;

    // 4. 李代数插值：result = T_a * exp(s * log(T_a^{-1} * T_b))
    SE2 relative = T_a.inverse() * T_b;

    auto log_rel = relative.log();        // 3维向量，例如 Eigen::Vector3d
    log_rel *= s;
    SE2 delta = SE2::exp(log_rel);
    result = T_a * delta;
    return true;
}

void Frontend::convertPoints( const SE2 & T_wi ){
    // 去畸变并且匹配完成之后，将点云再做一次修正
    SE2 deltaT = current_frame_->pose_.inverse() * T_wi; // T_wi_new.inverse() * T_wi_old

    // LOG(INFO) << "0: " << current_frame_->pts_[0].transpose();
    #pragma omp parallel for
    for (size_t i = 0; i < current_frame_->pts_.size(); ++i ) {
        current_frame_->pts_[i] = deltaT * current_frame_->pts_[i];
    }
    // LOG(INFO) << "1: " << current_frame_->pts_[0].transpose();
    // LOG(INFO) << deltaT.translation().transpose() << ", " << deltaT.so2().log() * 180. / M_PI;
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


