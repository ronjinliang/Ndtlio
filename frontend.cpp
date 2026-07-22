#include "2dNdtLIO(preintegration)/frontend.h"

#include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include <execution>
#include <opencv2/opencv.hpp>

namespace sad {

Frontend::Frontend( const std::string & fileName ){
    loadParamsFromYAML(fileName);
    imu_states_.clear();
}

bool Frontend::processScan( Scan2d::Ptr scan ){
    current_frame_ = std::make_shared<Frame>();  // 新的 frame，暂时还未传给 loop_closing 和 display
    current_frame_->id_ = frame_id_++;

    if (last_frame_) {
        // set pose from last frame
        current_frame_->pose_ = last_frame_->pose_ * motion_guess_;  // T_wl1 * T_l1l2
    }

    undistortAndGeneratePoints(scan);  // 去畸变同时转成点云

    // 利用scan matching来匹配地图
    // 第一帧无法匹配，且是 keyframe，直接加入到occupancy map
    if ( first_scan_ ) {
        first_scan_ = false;
        range_max_  = scan->range_max > opts_.max_distance_ ? opts_.max_distance_ : scan->range_max;
        range_min_  = scan->range_min;
        angle_max_  = scan->angle_max - opts_.angle_boarder_;
        angle_min_  = scan->angle_min + opts_.angle_boarder_;
        LOG(INFO) << "range max: " << range_max_ << ", range min: " << range_min_
                << ", angle max: " << angle_max_ * 180.0/M_PI << ", angle min: " << angle_min_ * 180.0/M_PI;
    } else {
        map_->getNdt().setSource(current_frame_);
        if ( eskf_ ) {  // eskf
            // ESKF 先验(初值)先转到雷达坐标系
            current_frame_->pose_ = eskf_->getNominalPose();  // 有点不是特别稳定
            map_->matchScan(current_frame_);  // 先匹配后融合   雷达坐标系
            eskf_->observeLidar(current_frame_->pose_);
            current_frame_->pose_ = eskf_->getNominalPose();
        } else if ( ieskf_ ) {   // ieskf
            ieskf_->updateUsingCustomObserve( [this]( const SE2 & init_pose, Mat8d & HT_Vinv_H, Vec8d & HT_Vinv_r ){
                map_->getNdt().conputeResidualAndJacobians(init_pose, HT_Vinv_H, HT_Vinv_r );  // 先计算矩阵和误差, 后融合
            });
            current_frame_->pose_ = ieskf_->getNominalPose();
        } else if ( preintegration_ ){  // IMU preintegration   // θ x y vx vy bg bax bay
            preintegration_->current_states_ = preintegration_->predict(preintegration_->last_states_);  // 可以考虑将这个作为 ndt 匹配的先验
            map_->matchScan(current_frame_);
            preintegration_->optimize(current_frame_->pose_);  // 里面已经赋值了
        } else {  // NDT LO
            map_->matchScan(current_frame_);
        }
    }

    bool is_kf = isKeyFrame();
    if ( is_kf ) {
        addKeyFrame(scan);  // scan 放入 occupancy map
    }

    display_->updateCurrentFrame(current_frame_, is_kf);

    if ( last_frame_ ) {
        motion_guess_ = last_frame_->pose_.inverse() * current_frame_->pose_;  // T_l1w * T_wl2 = T_l1l2
    }
    last_frame_ = current_frame_;
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
    } else if ( preintegration_ ) {
        res = preintegration_->integrate( imu );
        // imu_states_.push_back({imu->timestamp_, SE2(preintegration_->getDeltaR(preintegration_->bg_), preintegration_->getDeltaP(preintegration_->bg_, preintegration_->ba_))});
    }
    if ( imu_states_.size() == opts_.imu_states_buffer_size_ ) imu_states_.pop_front();  // 维持10个imu数据
    return res;
}

bool Frontend::processOdom( const std::shared_ptr<Odom> odom ){
    if ( eskf_ ) {
        return eskf_->observeOdom(odom);
    } else if ( ieskf_ ) {
        return ieskf_->observeOdom(odom);
    } else if ( preintegration_ ) {
        return preintegration_->setOdom(odom);;
    }
}

void Frontend::undistortAndGeneratePoints( Scan2d::Ptr scan ){  // 在 current_frame_->pose_ = last_frame_->pose_ * motion_guess_;  后面使用
    SE2 T_end = current_frame_->pose_;  // 这个位姿是猜的
    if ( eskf_ ) {
        T_end = eskf_->getNominalPose();
    } else if ( ieskf_ ) {
        T_end = ieskf_->getNominalPose();
    } else if ( preintegration_ ) {
        // T_end = SE2(preintegration_->getDeltaR(preintegration_->bg_), preintegration_->getDeltaP(preintegration_->bg_, preintegration_->ba_));
    }
    
    const float time_th = 0.5;
    int scan_num = scan->ranges.size();
    double lidar_begin_time = double(scan->header.stamp.sec) + double(scan->header.stamp.nanosec)*1e-9 - scan->time_increment * scan_num;
    double last_time = imu_states_.rbegin()->first;

    for ( int idx = 0; idx < scan_num; ++idx ) {
        double angle = scan->angle_min + idx * scan->angle_increment;  // 这个跟 range 对应的
        if ( !first_scan_ && (angle < angle_min_ || angle > angle_max_) ) continue;  // sb 玩意第一帧空的
        // if ( !first_scan_ && ( angle > 0.75*M_PI && angle < 1.25*M_PI) ) continue;   // 自己数据集去掉背后的数据  TUDO
        double range = scan->ranges[idx];
        if ( !first_scan_ && (range < range_min_ || range > range_max_) ) continue;
        double query_time = lidar_begin_time + idx * scan->time_increment;
        Vec2d raw_point( range * cos(angle), range * sin(angle) );
        if ( imu_states_.empty() ) {
            current_frame_->pts_.emplace_back(raw_point);
        } else {
            SE2 Ti = T_end;
            if (!poseInterp(query_time, last_time, Ti)) {
                LOG(INFO) << "interp false";
                Ti = T_end;
            }
            // SE2 deltaT = T_end.inverse() * Ti;
            SE2 deltaT = Ti.inverse() * T_end;   // 不知道为什么自己的代码要这样计算，高翔的是上面一行
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
    // T_wc2 = T_wc1 * T_c1c2
    // T_c1c2 = T_c1w * T_wc2
    SE2 delta_pose = last_keyframe_->pose_.inverse() * current_frame_->pose_;
    if (delta_pose.translation().norm() > opts_.kf_distance_ || fabs(delta_pose.so2().log()) > opts_.kf_angle_rad_ ) {
        return true;
    }
    return false;
}


/// 增加一个关键帧
void Frontend::addKeyFrame(Scan2d::Ptr scan){
    current_frame_->keyframe_id_ = keyframe_id_ ++;
    map_->addKeyframe(current_frame_);
    if ( (keyframe_id_ % (opts_.kf_add_scan_in_occu_+1)) == 0 ) map_->addScanInOccupancyMap(current_frame_, scan);  // 每x关键帧放一次
    map_->addScanInNdt(current_frame_);
    last_keyframe_ = current_frame_;
}


void Frontend::loadParamsFromYAML( const std::string & fileName ){
    YAML::Node config = YAML::LoadFile(fileName);
    opts_.kf_distance_            = config["frontend"]["kf_distance"].as<double>();
    opts_.kf_angle_deg_           = config["frontend"]["kf_angle_deg"].as<double>();
    opts_.kf_angle_rad_           = opts_.kf_angle_deg_ * M_PI / 180.;
    opts_.kf_add_scan_in_occu_    = config["frontend"]["kf_add_scan_in_occu"].as<int>();
    opts_.max_distance_           = config["frontend"]["max_distance"].as<double>();
    opts_.angle_boarder_          = config["frontend"]["angle_boarder"].as<double>() * M_PI / 180.0;
    opts_.imu_states_buffer_size_ = config["frontend"]["imu_states_buffer_size"].as<int>();
    opts_.display_                = config["frontend"]["display"].as<bool>();
    LOG(INFO) << "keyframe distance: " << opts_.kf_distance_;
    LOG(INFO) << "keyframe angle: " << opts_.kf_distance_;
    LOG(INFO) << "kf_add_scan_in_occu: " << opts_.kf_add_scan_in_occu_;
    LOG(INFO) << "max distance: " << opts_.max_distance_;
    LOG(INFO) << "angle boarder: " << opts_.angle_boarder_;
    LOG(INFO) << "imu states buffer size: " << opts_.imu_states_buffer_size_;
}



}


