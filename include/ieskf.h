#ifndef __IESKF_H
#define __IESKF_H

#include "../common/eigen_types.h"
#include "../common/imu.h"
#include "../common/odom.h"

namespace sad {
/**
 * states: px, py, vx, vy, theta, bg, bax, bay  8d
 */
class IESKF {
public:
    struct Options {
        Options() = default;
        double imu_dt_ = 0.01;  // 100 Hz
        
        int num_iterations_ = 3;  // 迭代次数
        double eps_ = 1e-3;  // 终止迭代的dx大小
        double info_ratio_ = 0.01;  // 每个点反馈的info因子
        
        // NOTE IMU噪声项都为离散时间，不需要再乘dt，可以由初始化器指定IMU噪声
        double gyro_var_ = 1e-5;       // 陀螺测量标准差
        double acce_var_ = 1e-2;       // 加计测量标准差
        double bias_gyro_var_ = 1e-6;  // 陀螺零偏游走标准差
        double bias_acce_var_ = 1e-4;  // 加计零偏游走标准差

        /// odom 噪声
        double odom_var_ = 0.5;

        /// 其他配置
        bool update_bias_gyro_ = true;  // 是否更新陀螺bias
        bool update_bias_acce_ = true;  // 是否更新加计bias

    };

    IESKF(Options opts);

    SE2 getNominalPose() const { return SE2(SO2(theta_), p_); }
    Vec8d getNominal();

    /**
     * 设置初始条件
     * @param opts 噪声
     * @param init_bg z 轴 gryo bias
     * @param init_ba x y 轴 acc bias
     */
    void setInitCondition( const Options & opts, const double & init_bg, const Vec2d & init_ba );

    void setSE2( SE2 pose ) { p_ = pose.translation(); theta_ = pose.so2().log(); }
    void setX( Vec2d p, Vec2d v, double theta, double bg, Vec2d ba, double timestamp );

    /// @brief IMU 递推
    /// @param imu 
    /// @return 
    bool predict( const IMUPtr imu );

    bool observeOdom( const std::shared_ptr<Odom> odom );

    /**
     * NDT观测函数，输入一个SE3 Pose, 返回本书(8.10)中的几个项
     * HT V^{-1} H
     * H^T V{-1} r
     * 二者都可以用求和的形式来做
     */
    using CustomObsFunc = std::function<void(const SE2& input_pose,
                                        Eigen::Matrix<double, 8, 8>& HT_Vinv_H,
                                        Eigen::Matrix<double, 8, 1>& HT_Vinv_r)>;

    /// 使用自定义观测函数更新滤波器
    bool updateUsingCustomObserve( CustomObsFunc obs );

private:
    void buildNoise();
    
    /// 更新名义状态变量
    void update();

private:
    Options opts_;

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
    
    // 轮速计观测 这就不用迭代了 松耦合得了
    Mat2d V_odom_ = Mat2d::Zero();
    Eigen::Matrix<double, 2, 8> H_odom_ = Eigen::Matrix<double, 2, 8>::Zero();
    Eigen::Matrix<double, 8, 2> H_odom_trans_ = Eigen::Matrix<double, 8, 2>::Zero();

    // 协方差矩阵
    Mat8d P_ = Mat8d::Identity();

    double last_timestamp_ = 0.0;
    bool first_lidar_ = true;

    Mat2d I22_ = Mat2d::Identity();
};


}


#endif
