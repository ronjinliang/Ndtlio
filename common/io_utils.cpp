#include "2dNdtLIO/common/io_utils.h"

#include <glog/logging.h>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>

namespace sad {

void RosbagIO::Go() {
    rosbag2_cpp::Reader reader;
    
    try {
        reader.open(bag_file_);
    } catch (const std::exception& e) {
        LOG(ERROR) << "cannot open " << bag_file_ << ": " << e.what();
        return;
    }

    LOG(INFO) << "running in " << bag_file_ << ", reg process func: " << process_func_.size();

    while (reader.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr serialized_msg = reader.read_next();
        auto topic_name = serialized_msg->topic_name;
        auto iter = process_func_.find(topic_name);
        // LOG(INFO) << topic_name;
        if (iter != process_func_.end()) {
            iter->second(serialized_msg);
        }
    }
    LOG(INFO) << "bag " << bag_file_ << " finished.";
}


/// 2D激光处理
RosbagIO &RosbagIO::AddScan2DHandle(const std::string &topic_name, Scan2DHandle f) {
    return AddHandle(topic_name, [f](rosbag2_storage::SerializedBagMessageSharedPtr msg) -> bool {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

        auto laser_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
        serialization.deserialize_message(&serialized_msg, laser_msg.get());
        
        if (laser_msg == nullptr) {
            LOG(INFO) << "cannot inst: laser scan msg";
            return false;
        }
        return f(laser_msg);
    });
}

RosbagIO &RosbagIO::AddMultiScanHandle( const std::string &topic_name, Scan2DHandle f){
    return AddHandle(topic_name, [this, f](rosbag2_storage::SerializedBagMessageSharedPtr msg) -> bool {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

        auto laser_msg = std::make_shared<sensor_msgs::msg::MultiEchoLaserScan>();
        rclcpp::Serialization<sensor_msgs::msg::MultiEchoLaserScan> serialization;
        serialization.deserialize_message(&serialized_msg, laser_msg.get());
        
        if (laser_msg == nullptr) {
            LOG(INFO) << "cannot inst: laser scan msg";
            return false;
        }
        return f(MultiToScan2d(laser_msg));
    });
}

RosbagIO &RosbagIO::AddOdomHandle( const std::string & topic_name, OdomHandle f ){
    return AddHandle( topic_name, [f, this](rosbag2_storage::SerializedBagMessageSharedPtr msg) ->bool {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        auto odom_msg = std::make_shared<origincar_msg::msg::Data>();
        rclcpp::Serialization<origincar_msg::msg::Data> serialization;
        serialization.deserialize_message(&serialized_msg, odom_msg.get());
        if ( odom_msg == nullptr ) {
            LOG(INFO) << "cannot inst: odom msg";
            return false;
        }
        std::shared_ptr<Odom> odom = std::make_shared<Odom>(odom_msg->x);
        return f(odom);
    } );
}

RosbagIO &RosbagIO::AddImuHandle(const std::string &topic_name, RosbagIO::ImuHandle f) {
    return AddHandle(topic_name, [&f, this, topic_name](rosbag2_storage::SerializedBagMessageSharedPtr msg) -> bool {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

        IMUPtr imu;
        if ( topic_name == "ivsensorimu" || topic_name == "/ivsensorimu" ) {
            auto imu_msg = std::make_shared<locsensor::msg::Ivsensorimu>();
            rclcpp::Serialization<locsensor::msg::Ivsensorimu> serialization;
            serialization.deserialize_message(&serialized_msg, imu_msg.get());
            auto stamp_seconds = double(imu_msg->header.stamp.sec) + double(imu_msg->header.stamp.nanosec) * 1e-9;
            imu = std::make_shared<IMU>(stamp_seconds,
                                  Vec3d(imu_msg->gyro_x* M_PI / 180.0, imu_msg->gyro_y* M_PI / 180.0, imu_msg->gyro_z* M_PI / 180.0),
                                  Vec3d(imu_msg->acce_x, imu_msg->acce_y, imu_msg->acce_z));
        } else {
            auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
            rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;  // 这里没写对的话读出来的数据是 0
            serialization.deserialize_message(&serialized_msg, imu_msg.get());
            auto stamp_seconds = double(imu_msg->header.stamp.sec) + double(imu_msg->header.stamp.nanosec) * 1e-9;
            imu = std::make_shared<IMU>(stamp_seconds,
                                    Vec3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z),
                                    Vec3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z));
        }
            
        
        return f(imu);
    });

}

}  // namespace sad
