#include "../include/eskf.h"
#include <glog/logging.h>

namespace sad {

ESKF::ESKF(Options options) : options_(options) { 
    buildNoise();

    P_ = Mat8d::Identity() * 1e-4; // 小初始值
    P_(4,4) = 0.01; // 角度初始不确定度
    P_(5,5) = 0.01; // 陀螺仪bias
    P_(6,6) = 0.01; // 加速度计bias
    P_(7,7) = 0.01;
    
    //   px  py  vx  vy  R   bg  bax bay
    //   0   1   2   3   4   5   6   7  
    //   
    //   0   0   1   0   0   0   0   0   0
    //   0   0   0   1   0   0   0   0   1
    H_odom_.block<2,2>(0,2) = Mat2d::Identity();
    H_odom_trans_ = H_odom_.transpose();
    
    //   px  py  vx  vy  R   bg  bax bay
    //   0   1   2   3   4   5   6   7  
    //   
    //   1   0   0   0   0   0   0   0   0
    //   0   1   0   0   0   0   0   0   1
    //   0   0   0   0   1   0   0   0   2
    H_lidar_.block<2,2>(0,0) = Mat2d::Identity();
    H_lidar_.block<1,1>(2,4) = Mat1d::Identity();
    H_lidar_trans_ = H_lidar_.transpose();
}

void ESKF::setInitCondition( 
    const Options & opts, const double & init_bg, const Vec2d & init_ba ) {
    options_ = opts;
    buildNoise();
    bg_ = init_bg;
    ba_ = init_ba;
}

void ESKF::setX( Vec2d p, Vec2d v, double theta, double bg, Vec2d ba, double timestamp ) {
    last_timestamp_ = timestamp;
    p_  = p;
    v_  = v;
    theta_  = theta;
    bg_ = bg;
    ba_ = ba;
}

Vec8d ESKF::getNominal() {
    Vec8d x;
    x.block<2,1>(0,0) = p_;
    x.block<2,1>(2,0) = v_;
    x.block<1,1>(4,0) = Mat1d(theta_);
    x.block<1,1>(5,0) = Mat1d(bg_);
    x.block<2,1>(6,0) = ba_;
    return x;
}

/// @brief IMU 递推
/// @param imu 
/// @return 
bool ESKF::predict( const IMUPtr imu ){
    double dt = imu->timestamp_ - last_timestamp_;
    // 时间间隔不对，可能是第一个IMU数据，没有历史信息
    if ( dt > (5*options_.imu_dt_) || dt < 0 ) {
        LOG(INFO) << "skip this imu data because dt " << dt;
        last_timestamp_ = imu->timestamp_;
        return false;
    }

    Vec2d a = Vec2d(imu->acce_.x(), imu->acce_.y()) - ba_;
    if ( imu->acce_.z() < 2.0 ) {  // 针对归一化之后的数据
        a *= 9.82;
    }
    dtheta_ = imu->gyro_.z() - bg_;
    double dg = dtheta_ * dt;

    // 1. 名义状态预测（非线性积分）
    // nominal state 名义状态积分，先用副本是防止更新过程中用到了 k+1 时刻的值，也就是预测之后的
    Vec2d temp_a = SO2(theta_) * a * dt;
    Vec2d new_p = p_ + v_*dt + 0.5 * temp_a * dt;
    Vec2d new_v = v_ + temp_a;
    double new_theta = theta_ + dg;

    // 限不限都没区别 因为直接积分，不是用的旋转矩阵，所以会超出周期
    // if ( new_theta > M_PI )  new_theta -= 2 * M_PI;
    // if ( new_theta < -M_PI ) new_theta += 2 * M_PI;

    p_ = new_p;
    v_ = new_v;
    theta_ = new_theta;

    // 2. 计算误差状态雅可比F
    Mat8d F = Mat8d::Identity();
    Mat2d R_mat = SO2(theta_).matrix();
    F.block<2,2>(0,2) = I22_*dt;
    // 不能直接对acce使用SO2::hat，自己推了一下，是这样子的
    F.block<2,1>(2,4) =   R_mat * Vec2d( -a.y(), a.x() ) * dt;
    F.block<2,2>(2,6) = - R_mat * dt;
    F.block<1,1>(4,5) = - Mat1d::Identity() * dt;

    // 3. 误差协方差矩阵
    P_ = F * P_.eval() * F.transpose() + Q_;
    
    last_timestamp_ = imu->timestamp_;
    return true;
}

/// @brief 编码器轮速计观测  我的数据集中的odom是跟imu同时发出来得
/// @param odom
/// @return 
bool ESKF::observeOdom( const std::shared_ptr<Odom> odom ){
    Eigen::Matrix<double, 8, 2> K = P_ * H_odom_trans_ * ( H_odom_ * P_ * H_odom_trans_ + V_odom_ ).inverse();
    
    // 轮速计测得的是机体坐标系 x 方向的速度
    // 转换到世界坐标系
    Vec2d v_w = SO2(theta_) * Vec2d( odom->v_, 0 );
    dx_ = K * ( v_w - v_ );

    P_ = ( Mat8d::Identity() - K * H_odom_ ) * P_;
    updateAndReset();  // 注入名义状态
    return true;
}

/// @brief lidar 递推
/// @param lidar
/// @return 
bool ESKF::observeLidar( const SE2 pose ){
    if ( first_lidar_ ) {
        first_lidar_ = false;
        theta_ = pose.so2().log();
        p_ = pose.translation();
        return true;
    }

    // # 1. 计算测量残差
    Vec3d innovation = Vec3d::Zero();
    innovation.head<2>() = pose.translation() - p_;
    innovation.tail<1>() = Mat1d(( SO2(theta_).inverse() * pose.so2() ).log());

    // # 2. 计算测量雅可比H
    // H_lidar_

    // # 3. 卡尔曼增益
    Eigen::Matrix<double, 8, 3> K = P_ * H_lidar_trans_ * ( H_lidar_ * P_ * H_lidar_trans_ + V_lidar_).inverse();
    
    // # 4. 误差状态更新
    dx_ += K * innovation;
    
    // # 5. 协方差更新
    P_ = ( Mat8d::Identity() - K * H_lidar_ ) * P_;
    
    // # 6. 注入误差到名义状态
    // # 7. 重置误差状态
    updateAndReset();

    return true;
}

void ESKF::updateAndReset(){
    p_ += dx_.block<2,1>(0,0);
    v_ += dx_.block<2,1>(2,0);
    theta_ += dx_(4,0);
    if ( options_.update_bias_gyro_ ) {
        bg_ += dx_(5,0);
    }
    if ( options_.update_bias_acce_ ) {
        ba_ += dx_.block<2,1>(6,0);
    }

    dx_.setZero();
}


void ESKF::buildNoise() {
    double eta_v     = options_.acce_var_;
    double eta_theta  = options_.gyro_var_;
    double eta_bg     = options_.bias_gyro_var_;
    double eta_ba     = options_.bias_acce_var_;
    
    double eta_v2     = eta_v * eta_v;
    double eta_theta2 = eta_theta * eta_theta;
    double eta_bg2    = eta_bg * eta_bg;
    double eta_ba2    = eta_ba * eta_ba;

    Q_.diagonal() <<
        0.0, 0.0,            // px, py: 无过程噪声（可选：加小值如1e-8防止数值问题）
        eta_v2, eta_v2,      // vx, vy
        eta_theta2,          // θ (gyro测量影响旋转)
        eta_bg2,             // bg
        eta_ba2, eta_ba2;    // bax, bay

    double o2 = options_.odom_var_ * options_.odom_var_;
    V_odom_.diagonal() << o2, o2;

    double lidat_pos2   = options_.lidar_pos_noise_ * options_.lidar_pos_noise_;
    double lidar_theta2 = options_.lidar_ang_noise_ * options_.lidar_ang_noise_;
    V_lidar_.diagonal() << lidat_pos2, lidat_pos2, lidar_theta2;
}

}
