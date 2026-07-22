#include "2dNdtLIO(preintegration)/incrementalNDTLO.h"
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace sad {

IncrementalNDTLO::IncrementalNDTLO( const std::string & fileName, int with_imu ) {
    YAML::Node config = YAML::LoadFile(fileName);

    frontend_ = std::make_shared<Frontend>( fileName );
    map_ = std::make_shared<Map>( fileName );
    display_ = std::make_shared<Display>( config["main"]["display_maxsize"].as<int>() );
    
    frontend_->setDisplay(display_);
    frontend_->setMap(map_);

    if ( with_imu == 1 ) {
        eskf_ = std::make_shared<ESKF>(ESKF::Options());
        frontend_->setESKF(eskf_);
    } else if ( with_imu == 2 ) {
        ieskf_ = std::make_shared<IESKF>(IESKF::Options());
        frontend_->setIESKF(ieskf_);
    } else if ( with_imu == 3 ) {
        preintegration_ = std::make_shared<IMUPreintegration>();
        frontend_->setIMUPreintegration(preintegration_);
    }

    display_->setMap(map_);

    imu_dt_                 = config["imu"]["imu_dt"].as<double>();
    
    gyro_var_               = config["imu"]["gyro_var"].as<double>();
    acce_var_               = config["imu"]["acce_var"].as<double>();
    bias_gyro_var_          = config["imu"]["bias_gyro_var"].as<double>();
    bias_acce_var_          = config["imu"]["bias_acce_var"].as<double>();
    odom_var_               = config["imu"]["odom_var"].as<double>();

    eskf_lidar_pos_noise_   = config["imu"]["eskf"]["lidar_pos_noise"].as<double>();
    eskf_lidar_ang_noise_   = config["imu"]["eskf"]["lidar_ang_noise"].as<double>();

    ieskf_num_iterations_   = config["imu"]["ieskf"]["num_iterations"].as<int>();
    ieskf_eps_              = config["imu"]["ieskf"]["eps"].as<double>();
    ieskf_info_ratio_       = config["imu"]["ieskf"]["info_ratio"].as<double>();
    ieskf_update_bias_gyro_ = config["imu"]["ieskf"]["update_bias_gyro"].as<bool>();
    ieskf_update_bias_acce_ = config["imu"]["ieskf"]["update_bias_acce"].as<bool>();

    pre_ndt_pos_noise_      = config["imu"]["preintegration"]["ndt_pos_noise"].as<double>();;
    pre_ndt_ang_noise_      = config["imu"]["preintegration"]["ndt_ang_noise"].as<double>();;
    pre_update_bias_gyro_   = config["imu"]["preintegration"]["update_bias_gyro"].as<bool>();
    pre_update_bias_acce_   = config["imu"]["preintegration"]["update_bias_acce"].as<bool>();
    pre_info_wright_        = config["imu"]["preintegration"]["info_wright"].as<double>();
}

bool IncrementalNDTLO::initIMU( IMUPtr imu ){
    if ( !static_imu_init_.InitSuccess() && !imu_init_success_ ) {  // TUDO 自己的数据集暂时先注释掉这个
        static_imu_init_.AddIMU(*imu);
        return false;
    }
    if ( !imu_init_success_ ) {
        imu_init_success_ = true;
        
        double gyro_var      = gyro_var_;
        double acce_var      = acce_var_;
        double bias_gyro_var = bias_gyro_var_;
        double bias_acce_var = bias_acce_var_;
        if ( gyro_var < 0 ) gyro_var = sqrt(static_imu_init_.GetCovGyro()(2));
        if ( acce_var < 0 ) acce_var = sqrt(static_imu_init_.GetCovAcce()(0));
        // 通常偏差游走是测量噪声的1/10到1/100
        if ( bias_gyro_var < 0 ) bias_gyro_var = 0.01 * gyro_var;
        if ( bias_acce_var < 0 ) bias_acce_var = 0.01 * acce_var;
        double bg = static_imu_init_.GetInitBg()(2);
        Vec2d ba = Vec2d(
            static_imu_init_.GetInitBa()(0) + static_imu_init_.GetGravity()(0),
            static_imu_init_.GetInitBa()(1) + static_imu_init_.GetGravity()(1));
        
        // 自己的数据集直接设定
        // double bg = -0.0270514;
        // Vec2d ba = Vec2d(-0.0506324, 0.453892);

        LOG(INFO) << "gyro var: " << gyro_var << ", acce var: " << acce_var 
                << ", bias gyro var: " << bias_gyro_var << ", bias acce var: " << bias_acce_var
                << ", bg: " << bg << ", ba: " << ba.transpose();

        if ( eskf_ ) {
            ESKF::Options opts;
            opts.imu_dt_          = imu_dt_;
            opts.gyro_var_        = gyro_var;
            opts.acce_var_        = acce_var;
            opts.bias_gyro_var_   = bias_gyro_var;
            opts.bias_acce_var_   = bias_acce_var;
            opts.odom_var_        = odom_var_;
            opts.lidar_pos_noise_ = eskf_lidar_pos_noise_;
            opts.lidar_ang_noise_ = eskf_lidar_ang_noise_;
            eskf_->setInitCondition(opts, bg, ba);
            LOG(INFO) << "ESKF: imu init finished.";
        } else if ( ieskf_ ) {
            IESKF::Options opts;
            opts.imu_dt_         = imu_dt_;
            opts.num_iterations_ = ieskf_num_iterations_;  // 迭代次数
            opts.eps_            = ieskf_eps_;             // 终止迭代的dx大小
            opts.info_ratio_     = ieskf_info_ratio_;

            opts.gyro_var_       = gyro_var;
            opts.acce_var_       = acce_var;
            opts.bias_gyro_var_  = bias_gyro_var;
            opts.bias_acce_var_  = bias_acce_var;

            opts.odom_var_         = odom_var_;
            opts.update_bias_gyro_ = ieskf_update_bias_gyro_;
            opts.update_bias_acce_ = ieskf_update_bias_acce_;

            ieskf_->setInitCondition(opts, bg, ba);
            LOG(INFO) << "IESKF: imu init finished.";
        } else if ( preintegration_ ) {
            IMUPreintegration::Options opts;
            opts.imu_dt_           = imu_dt_;
            opts.noise_gyro_       = gyro_var;
            opts.noise_acce_       = acce_var;
            opts.bias_gyro_var_    = bias_gyro_var;
            opts.bias_acce_var_    = bias_acce_var;

            opts.odom_var_         = odom_var_;

            opts.ndt_pos_noise_    = pre_ndt_pos_noise_;
            opts.ndt_ang_noise_    = pre_ndt_ang_noise_;
            opts.update_bias_gyro_ = pre_update_bias_gyro_;
            opts.update_bias_acce_ = pre_update_bias_acce_;
            opts.info_wright_      = pre_info_wright_;

            preintegration_->setInitCondition(opts, bg, ba);
            LOG(INFO) << "IMU Preintegration: imu init finished.";
        }

        return frontend_->processIMU(imu);
    } else {
        return frontend_->processIMU(imu);
    }
}

void IncrementalNDTLO::close(){
    display_->close();
}


}
