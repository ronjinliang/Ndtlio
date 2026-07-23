#include <yaml-cpp/yaml.h>
#include <glog/logging.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "common/static_imu_init.h"

class ProcessImuNode : public rclcpp::Node {
public:
    ProcessImuNode( const std::string & node_name ) : Node(node_name) {
        std::string config_file_;
        this->declare_parameter<std::string>("config_file", "/home/lrj/origincar_ws/src/Ndtlio/config/process_imu.yaml");
        this->get_parameter("config_file", config_file_);
        LOG(INFO) << "using config: " << config_file_;

        YAML::Node config = YAML::LoadFile(config_file_);
        std::string imu_topic = config["imu_topic"].as<std::string>();

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10, std::bind(&ProcessImuNode::imuCallback, this, std::placeholders::_1));
    }

    ~ProcessImuNode() { }

private:
    double timeStamp( const int & sec, const int & nanosec ){
        return double(sec) + double(nanosec) * 1e-9;
    }

    void imuCallback( const sensor_msgs::msg::Imu::SharedPtr msg ) {
        IMUPtr imu      = std::make_shared<sad::IMU>();
        imu->timestamp_ = timeStamp(msg->header.stamp.sec, msg->header.stamp.nanosec);
        imu->acce_      = Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        imu->gyro_      = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

        if ( !static_imu_init_.InitSuccess() ) {
            static_imu_init_.AddIMU(*imu);
            return;
        }

        Vec3d acc_mean = static_imu_init_.GetMeanAcce();
        double g_norm = acc_mean.norm();
        Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(acc_mean.normalized(), -Vec3d::UnitZ());
        Eigen::Matrix3d R = q.toRotationMatrix();
        LOG(INFO) << "acc_world = R * acc_imu";
        LOG(INFO) << "Static projection matrix:\n" << R;
        LOG(INFO) << (R * acc_mean).transpose();

        double gyro_var = sqrt(static_imu_init_.GetCovGyro()(2));
        double acce_var = sqrt(static_imu_init_.GetCovAcce()(0));
        // 通常偏差游走是测量噪声的1/10到1/100
        double bias_gyro_var = 0.01 * gyro_var;
        double bias_acce_var = 0.01 * acce_var;

        Vec3d g = static_imu_init_.GetGravity();
        double bg = static_imu_init_.GetInitBg()(2);
        Vec2d ba = Vec2d( static_imu_init_.GetInitBa()(0), static_imu_init_.GetInitBa()(1));

        LOG(INFO) << "gyro var: " << gyro_var;
        LOG(INFO) << "acce var: " << acce_var;
        LOG(INFO) << "bias gyro var: " << bias_gyro_var;
        LOG(INFO) << "bias acce var: " << bias_acce_var;
        LOG(INFO) << "bg: " << bg;
        LOG(INFO) << "ba: " << ba.transpose();
                
        LOG(INFO) << "Finished! Now exiting.";
        rclcpp::shutdown();
    }


private:
    sad::StaticIMUInit static_imu_init_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = nullptr;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ProcessImuNode>("process_imu_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

