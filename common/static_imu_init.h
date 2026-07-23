#ifndef __STATIC_IMU_INIT_H
#define __STATIC_IMU_INIT_H

#include "../common/eigen_types.h"
#include "../common/imu.h"
#include "../common/odom.h"
#include "../common/math_utils.h"

#include <deque>
#include <glog/logging.h>

namespace sad {
    
/**
 * IMU水平静止状态下初始化器
 * 使用方法：调用AddIMU, AddOdom添加数据，使用InitSuccess获取初始化是否成功
 * 成功后，使用各Get函数获取内部参数
 *
 * 初始化器在每次调用AddIMU时尝试对系统进行初始化。在有odom的场合，初始化要求odom轮速读数接近零；没有时，假设车辆初期静止。
 * 初始化器收集一段时间内的IMU读数，按照书本3.5.4节估计初始零偏和噪声参数，提供给ESKF或者其他滤波器
 */
class StaticIMUInit{
public:
    struct Options {
        Options(){}
        double init_time_seconds_ = 10.0;      // 静止时间
        int init_imu_queue_max_size_ = 2000;   // 初始化IMU队列的最大长度
        int static_odom_pulse_ = 5;            // 静止时轮速计输出噪声
        double max_static_gyro_var = 0.2;      // 静态下陀螺仪测量方差
        double max_static_acce_var = 0.05;     // 静态下加速度计测量方差
        double gravity_norm_ = 9.81;            // 重力大小
        bool use_speed_for_static_checking_ = true;  // 是否使用 ODOM 来判断车辆静止
    };

    // 构造函数
    StaticIMUInit(Options options = Options()) : options_(options) {}

    // 添加 IMU 数据
    bool AddIMU(const IMU& imu) {
        if (init_success_) {
            return true;
        }

        if (options_.use_speed_for_static_checking_ && !is_static_) {
            LOG(WARNING) << "等待车辆静止";
            init_imu_deque_.clear();
            return false;
        }

        if (init_imu_deque_.empty()) {
            // 记录初始静止时间
            init_start_time_ = imu.timestamp_;
        }

        // 记入初始化队列
        init_imu_deque_.push_back(imu);
        double init_time = imu.timestamp_ - init_start_time_;  // 初始化经过时间
        if (init_time > options_.init_time_seconds_) {
            // 尝试初始化逻辑
            TryInit();
        }

        // 维持初始化队列长度
        while (init_imu_deque_.size() > options_.init_imu_queue_max_size_) {
            init_imu_deque_.pop_front();
        }

        current_time_ = imu.timestamp_;
        return false;
    }

    // 添加 轮速数据
    bool AddOdom(const Odom & odom){
        // 判断车辆是否静止
        if (init_success_) {
            return true;
        }

        if (odom.left_pulse_ < options_.static_odom_pulse_ && odom.right_pulse_ < options_.static_odom_pulse_) {
            is_static_ = true;
        } else {
            is_static_ = false;
        }

        current_time_ = odom.timestamp_;
        return true;
    }

    // 判断初始化是否成功
    bool InitSuccess() const { return init_success_; }

    Vec3d GetCovGyro() const { return cov_gyro_; }
    Vec3d GetCovAcce() const { return cov_acce_; }
    Vec3d GetInitBg() const { return init_bg_; }
    Vec3d GetInitBa() const { return init_ba_; }
    Vec3d GetGravity() const { return gravity_; }
    Vec3d GetMeanAcce() const { return mean_acce_; }

private:
    // 尝试对系统初始化
    bool TryInit() {
        if (init_imu_deque_.size() < 10) {
            return false;
        }

        // 计算均值和方差
        Vec3d mean_gyro, mean_acce;
        math::ComputeMeanAndCovDiag(init_imu_deque_, mean_gyro, cov_gyro_, [](const IMU& imu) { return imu.gyro_; });
        math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_, [this](const IMU& imu) { return imu.acce_; });

        mean_acce_ = mean_acce;

        // 以acce均值为方向，取9.8长度为重力
        LOG(INFO) << "mean acce: " << mean_acce.transpose();
        gravity_ = -mean_acce / mean_acce.norm() * options_.gravity_norm_;

        // 重新计算加计的协方差
        math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_,
                                    [this](const IMU& imu) { return imu.acce_ + gravity_; });

        // 检查IMU噪声
        if (cov_gyro_.norm() > options_.max_static_gyro_var) {
            LOG(ERROR) << "陀螺仪测量噪声太大" << cov_gyro_.norm() << " > " << options_.max_static_gyro_var;
            return false;
        }

        if (cov_acce_.norm() > options_.max_static_acce_var) {
            LOG(ERROR) << "加计测量噪声太大" << cov_acce_.norm() << " > " << options_.max_static_acce_var;
            return false;
        }

        // 估计测量噪声和零偏
        init_bg_ = mean_gyro;
        init_ba_ = mean_acce;

        LOG(INFO) << "IMU 初始化成功，初始化时间= " << current_time_ - init_start_time_ << ", bg = " << init_bg_.transpose()
                << ", ba = " << init_ba_.transpose() << ", gyro sq = " << cov_gyro_.transpose()
                << ", acce sq = " << cov_acce_.transpose() << ", grav = " << gravity_.transpose()
                << ", norm: " << gravity_.norm();
        LOG(INFO) << "mean gyro: " << mean_gyro.transpose() << " acce: " << mean_acce.transpose();
        init_success_ = true;
        return true;
    }

    Options options_;
    bool init_success_ = false;

    Vec3d cov_gyro_ = Vec3d::Zero();    // 陀螺仪测量噪声协方差
    Vec3d cov_acce_ = Vec3d::Zero();    // 加速度计测量噪声协方差
    Vec3d init_bg_ = Vec3d::Zero();     // 陀螺仪初始零偏
    Vec3d init_ba_ = Vec3d::Zero();     // 加速度计初始零偏
    Vec3d gravity_ = Vec3d::Zero();     // 重力
    Vec3d mean_acce_ = Vec3d::Zero();

    bool is_static_ = true;             // 判断车是否静止
    std::deque<IMU> init_imu_deque_;    // 初始化用的数据
    double current_time_ = 0.0;         // 当前时间
    double init_start_time_ = 0.0;      // 静止初始时间

};


}



#endif // __STATIC_IMU_INIT_H
