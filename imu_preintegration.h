#ifndef IMU_PREINTEGRATION_H
#define IMU_PREINTEGRATION_H

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/odom.h"

namespace sad {

class IMUPreintegration {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /// 参数配置项
    /// 初始的零偏需要设置，其他可以不改
    struct Options {
        Options() {}
        double imu_dt_ = 0.01;

        double info_wright_ = 0.1;

        double noise_gyro_ = 1e-2;      // 陀螺仪噪声，标准差
        double noise_acce_ = 1e-1;      // 加速度计噪声，标准差
        
        double bias_gyro_var_ = 1e-2;           // 陀螺零偏游走标准差
        double bias_acce_var_ = 1e-2;           // 加计零偏游走标准差
        Mat1d bg_rw_info_ = Mat1d::Identity();  // 陀螺随机游走信息阵
        Mat2d ba_rw_info_ = Mat2d::Identity();  // 加计随机游走信息阵

        /// odom 噪声
        double odom_var_ = 0.5;
        Mat2d odom_info_ = Mat2d::Identity();

        double ndt_pos_noise_ = 0.1;                   // NDT位置方差
        double ndt_ang_noise_ = 2.0 * M_PI / 180.0;  // NDT角度方差
        Mat3d ndt_info_ = Mat3d::Identity();

        bool update_bias_gyro_ = true;
        bool update_bias_acce_ = true;
    };
    
    IMUPreintegration() = default;
    IMUPreintegration( Options options, const double & init_bg, const Vec2d & init_ba );

    void setInitCondition( const Options & opts, const double & init_bg, const Vec2d & init_ba );

    /**
     * 插入新的IMU数据
     * @param imu   imu 数据
     */
    bool integrate( const IMUPtr imu );

    bool setOdom( const std::shared_ptr<Odom> odom );

    /**
     * 从某个起始点开始预测积分之后的状态  theta p v bg ba
     * @param start 起始时时刻状态
     * @return  预测的状态
     */
    Vec8d predict( const Vec8d &start ) const;

    /// 获取修正之后的观测量，bias可以与预积分时期的不同，会有一阶修正
    SO2 getDeltaR(const double &bg);
    Vec2d getDeltaV(const double &bg, const Vec2d &ba);
    Vec2d getDeltaP(const double &bg, const Vec2d &ba);

    /// 执行预积分+NDT pose优化   odom
    void optimize( SE2 & ndt_pose );
private:
    void reset();
public:
    Options opts_;

    double dt_ = 0.;    // 预积分时间
    double last_timestamp_ = 0.0;
    // 预积分观测量
    double dtheta_ = 0.0;
    Vec2d dv_ = Vec2d::Zero();
    Vec2d dp_ = Vec2d::Zero();
    // 零偏
    double bg_ = 0.0;
    Vec2d ba_ = Vec2d::Zero();

    Mat5d cov_ = Mat5d::Zero();    // P 噪声协方差矩阵 ∑ = A ∑ A^T + B Cov(n_d) B^T (4.31)
    Mat3d noise_gyro_acce_ = Mat3d::Zero();  // 测量噪声矩阵

    // 零偏更新的5个雅可比矩阵   对 bg 求导是 2x1
    double dR_dbg_ = 0.0;  // 1x1
    Mat2d dv_dba_ = Mat2d::Zero();  // 2x2
    Vec2d dv_dbg_ = Vec2d::Zero();  // 2x1
    Mat2d dp_dba_ = Mat2d::Zero();  // 2x2
    Vec2d dp_dbg_ = Vec2d::Zero();  // 2x1

    std::shared_ptr<Odom> last_odom_    = nullptr;
    std::shared_ptr<Odom> current_odom_ = nullptr;
    Vec2d last_odom_speed_              = Vec2d::Zero();
    Vec2d current_odom_speed_           = Vec2d::Zero();

    // optimize
    // 0 1 2 3  4  5   6   7
    // θ x y vx vy bg bax bay
    Vec8d last_states_ = Vec8d::Zero();
    Vec8d current_states_ = Vec8d::Zero();    // 上一时刻状态与本时刻状态
    Mat8d prior_info_ = Mat8d::Identity();  // 先验约束

};

}

#endif  // IMU_PREINTEGRATION_H
