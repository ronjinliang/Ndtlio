#include "../include/ieskf.h"
#include <glog/logging.h>


namespace sad {

IESKF::IESKF(Options opts) : opts_(opts) { 
    buildNoise();

    P_ = Mat8d::Identity() * 1e-4; // 小初始值
    P_(4,4) = 0.01; // 角度初始不确定度
    P_(5,5) = 0.01; // 陀螺仪bias
    P_(6,6) = 0.01; // 加速度计bias
    P_(7,7) = 0.01;

    dx_ = Vec8d::Zero();  // 误差状态初始化为零向量
    
    //   px  py  vx  vy  R   bg  bax bay
    //   0   1   2   3   4   5   6   7  
    //   
    //   0   0   1   0   0   0   0   0   0
    //   0   0   0   1   0   0   0   0   1
    H_odom_.block<2,2>(0,2) = Mat2d::Identity();
    H_odom_trans_ = H_odom_.transpose();
}

void IESKF::setInitCondition( 
    const Options & opts, const double & init_bg, const Vec2d & init_ba ) {
    opts_ = opts;
    buildNoise();
    bg_ = init_bg;
    ba_ = init_ba;
}

void IESKF::setX( Vec2d p, Vec2d v, double theta, double bg, Vec2d ba, double timestamp ) {
    last_timestamp_ = timestamp;
    p_  = p;
    v_  = v;
    theta_ = theta;
    bg_ = bg;
    ba_ = ba;
}

Vec8d IESKF::getNominal() {
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
bool IESKF::predict( const IMUPtr imu ){
    double dt = imu->timestamp_ - last_timestamp_;
    // 时间间隔不对，可能是第一个IMU数据，没有历史信息
    if ( dt > (5*opts_.imu_dt_) || dt < 0 ) {
        LOG(INFO) << "skip this imu data because dt " << dt;
        last_timestamp_ = imu->timestamp_;
        return false;
    }

    Vec2d imu_acce(imu->acce_.x(), imu->acce_.y());
    Vec2d a = imu_acce - ba_;
    if ( imu->acce_.z() < 2.0 ) {  // 针对归一化之后的数据
        a *= 9.82;
    }
    double dg = ( imu->gyro_.z() - bg_ ) * dt;

    // 1. 名义状态预测（非线性积分）
    // nominal state 名义状态积分，先用副本是防止更新过程中用到了 k+1 时刻的值，也就是预测之后的
    
    Vec2d temp_a = SO2(theta_) * a * dt;
    Vec2d new_p = p_ + v_*dt + 0.5 * temp_a * dt;
    Vec2d new_v = v_ + temp_a;
    double new_theta = theta_ + dg;

    p_ = new_p;
    v_ = new_v;
    theta_ = new_theta;

    // 2. 计算误差状态雅可比F
    Mat8d F = Mat8d::Identity();
    Mat2d R_mat = SO2(theta_).matrix();
    F.block<2,2>(0,2) = I22_ * dt;
    // 不能直接对acce使用SO2::hat，自己推了一下，是这样子的
    F.block<2,1>(2,4) =   R_mat * Vec2d( -a.y(), a.x() ) * dt;
    F.block<2,2>(2,6) = - R_mat * dt;
    // F(4,4) = 1 (identity), already set by Mat8d::Identity()
    F.block<1,1>(4,5) = - Mat1d::Identity() * dt;

    // 3. 误差协方差矩阵
    P_ = F * P_.eval() * F.transpose() + Q_;
    
    last_timestamp_ = imu->timestamp_;
    return true;
}

/// @brief 编码器轮速计观测  我的数据集中的odom是跟imu同时发出来得
/// @param odom
/// @return 
bool IESKF::observeOdom( const std::shared_ptr<Odom> odom ){
    Eigen::Matrix<double, 8, 2> K = P_ * H_odom_trans_ * ( H_odom_ * P_ * H_odom_trans_ + V_odom_ ).inverse();
    
    // 轮速计测得的是机体坐标系 x 方向的速度
    // 转换到世界坐标系
    Vec2d v_w = SO2(theta_) * Vec2d( odom->v_, 0 );
    dx_ = K * ( v_w - v_ );

    P_ = ( Mat8d::Identity() - K * H_odom_ ) * P_;
    
    // 注入名义状态
    p_ += dx_.block<2,1>(0,0);
    v_ += dx_.block<2,1>(2,0);
    theta_ += dx_(4,0);
    if ( opts_.update_bias_gyro_ ) {
        bg_ += dx_(5,0);
    }
    if ( opts_.update_bias_acce_ ) {
        ba_ += dx_.block<2,1>(6,0);
    }

    /// 对P阵进行投影，参考式(3.63)
    double J = 1 - 0.5*dx_(4);
    double J2 = J*J;
    P_.block<1,8>(4,0) = J2 * P_.block<1,8>(4,0);
    P_.block<8,1>(0,4) = J2 * P_.block<8,1>(0,4);
    dx_.setZero();
    return true;
}

bool IESKF::updateUsingCustomObserve( CustomObsFunc obs){
    double start_theta = theta_;
    Mat8d HT_Vinv_H;
    Vec8d HT_Vinv_r;
    Eigen::Matrix<double, 8, 3> K;
    Mat8d Pk = Mat8d::Zero();
    Mat8d Qk = Mat8d::Zero();
    double J = 0.0;
    double J2 = 0.0;

    for (int iter = 0; iter < opts_.num_iterations_; ++iter){
        // 调用 obs function   数值问题，是里面计算 HT_Vinv_H, HT_Vinv_r 的 J 矩阵出现数值问题
        obs( SE2( theta_, p_ ), HT_Vinv_H, HT_Vinv_r );  // 用名义状态给 ndt 构建矩阵
        HT_Vinv_H = HT_Vinv_H * opts_.info_ratio_;
        HT_Vinv_r = HT_Vinv_r * opts_.info_ratio_;

        // 投影 p
        // Mat8d J = Mat8d::Identity();
        // J(4,4) = 1.0 - 0.5 * dx_(4,0);
        // Pk = J * P_ * J.transpose();

        Pk = P_;  // 先赋值再计算投影
        J = 1 - 0.5*dx_(4);  // 非常小的值
        J2 = J*J;
        Pk.block<1,8>(4,0) = J2 * P_.block<1,8>(4,0);
        Pk.block<8,1>(0,4) = J2 * P_.block<8,1>(0,4);

        Qk = ( Pk.inverse() + HT_Vinv_H ).inverse();  // 这个记作中间变量，最后更新时可以用   直接这样会出现 nan
        dx_ = Qk * HT_Vinv_r;
        if ( dx_.hasNaN() ) continue;
        // LOG(INFO) << "iter " << iter << " dx = " << dx_.transpose() << ", dxn: " << dx_.norm();

        // 合入名义变量
        update();

        if (dx_.norm() < opts_.eps_) {
            break;
        }
    }
    // update P
    P_ = ( Mat8d::Identity() - Qk * HT_Vinv_H ) * Pk;

    // project P
    J = 1 - 0.5*dx_(4);
    J2 = J*J;
    P_.block<1,8>(4,0) = J2 * P_.block<1,8>(4,0);
    P_.block<8,1>(0,4) = J2 * P_.block<8,1>(0,4);
    dx_.setZero();

    return true;
}

void IESKF::buildNoise() {
    double eta_v = opts_.acce_var_;
    double eta_theta = opts_.gyro_var_;
    double eta_bg = opts_.bias_gyro_var_;
    double eta_ba = opts_.bias_acce_var_;
    
    double eta_v2 = eta_v * eta_v;
    double eta_theta2 = eta_theta * eta_theta;
    double eta_bg2 = eta_bg * eta_bg;
    double eta_ba2 = eta_ba * eta_ba;

    Q_.diagonal() <<
        0.0, 0.0,          // px, py: 无过程噪声（可选：加小值如1e-8防止数值问题）
        eta_v2, eta_v2,      // vx, vy
        eta_theta2,          // θ (gyro测量影响旋转)
        eta_bg2,             // bg
        eta_ba2, eta_ba2;    // bax, bay

    
    double o2 = opts_.odom_var_ * opts_.odom_var_;
    V_odom_.diagonal() << o2, o2;
}

/// 更新名义状态变量
void IESKF::update() {
    p_ += dx_.block<2,1>(0,0);
    v_ += dx_.block<2,1>(2,0);
    theta_ += dx_(4);
    if ( opts_.update_bias_gyro_ ) {
        bg_ += dx_(5);
    }
    if ( opts_.update_bias_acce_ ) {
        ba_ += dx_.block<2,1>(6,0);
    }

}

}
