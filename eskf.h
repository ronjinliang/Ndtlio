#ifndef __ESKF_H
#define __ESKF_H

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/odom.h"
#include "common/math_utils.h"

namespace sad {
/**
 * states: px, py, vx, vy, theta, bg, bax, bay  8d
 */
class ESKF {
public:
    struct Options {
        Options(){};
        double imu_dt_ = 0.01;  // 100 Hz
        
        // NOTE IMU噪声项都为离散时间，不需要再乘dt，可以由初始化器指定IMU噪声
        double gyro_var_ = 1e-5;       // 陀螺测量标准差
        double acce_var_ = 1e-2;       // 加计测量标准差
        double bias_gyro_var_ = 1e-6;  // 陀螺零偏游走标准差
        double bias_acce_var_ = 1e-4;  // 加计零偏游走标准差

        /// odom 噪声
        double odom_var_ = 0.5;

        /// Lidar 观测噪声
        double lidar_pos_noise_ = 0.1;                   // Lidar 位置噪声
        double lidar_ang_noise_ = 0.1 * math::kDEG2RAD;  // Lidar 旋转噪声

        /// 其他配置
        bool update_bias_gyro_ = true;  // 是否更新陀螺bias
        bool update_bias_acce_ = true;  // 是否更新加计bias
    };

    ESKF(Options options);

    SE2 getNominalPose() const { return SE2( SO2(theta_), p_ ); }
    Vec8d getNominal();

    /**
     * 设置初始条件
     * @param opts 噪声
     * @param init_bg z 轴 gryo bias
     * @param init_ba x y 轴 acc bias
     */
    void setInitCondition( const Options & opts, const double & init_bg, const Vec2d & init_ba );

    /**
     * 设置名义状态变量
     */
    void setX( Vec2d p, Vec2d v, double theta, double bg, Vec2d ba, double timestamp );

    /// @brief IMU 递推
    /// @param imu 
    /// @return 
    bool predict( const IMUPtr imu );

    /// @brief 编码器轮速计观测
    /// @param odom
    /// @return 
    bool observeOdom( const std::shared_ptr<Odom> odom );

    /// @brief lidar 观测
    /// @param lidar
    /// @return 
    bool observeLidar( const SE2 pose );

    void updateAndReset();

private:
    void buildNoise();

private:
    /// 名义状态变量
    Vec2d p_ = Vec2d::Zero();
    Vec2d v_ = Vec2d::Zero();
    double theta_ = 0.0;
    double bg_ = 0.0;
    Vec2d ba_ = Vec2d::Zero();

    // 误差状态变量
    Vec8d dx_ = Vec8d::Zero();

    // 过程噪声
    Mat8d Q_ = Mat8d::Identity();

    // 轮速计观测噪声矩阵
    Mat2d V_odom_ = Mat2d::Zero();
    Eigen::Matrix<double, 2, 8> H_odom_ = Eigen::Matrix<double, 2, 8>::Zero();
    Eigen::Matrix<double, 8, 2> H_odom_trans_ = Eigen::Matrix<double, 8, 2>::Zero();

    // 雷达观测噪声矩阵
    Mat3d V_lidar_ = Mat3d::Zero();
    // 雷达观测的 H 矩阵
    Eigen::Matrix<double, 3, 8> H_lidar_ = Eigen::Matrix<double, 3, 8>::Zero();
    Eigen::Matrix<double, 8, 3> H_lidar_trans_ = Eigen::Matrix<double, 8, 3>::Zero();

    // 协方差矩阵
    Mat8d P_ = Mat8d::Identity();

    double last_timestamp_ = 0.0;
    bool first_lidar_ = true;

    Mat2d I22_ = Mat2d::Identity();

    Options options_;

};


}


#endif
