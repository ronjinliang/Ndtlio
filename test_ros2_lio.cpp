#include <yaml-cpp/yaml.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "2dNdtLIO/include/incrementalNDTLO.h"
#include "2dNdtLIO/ros2node/ndt_lio_node.h"

DEFINE_string(config_file, "/home/lrj/lidar_slam/src/2dNdtLIO/config/ndt_params_origincar.yaml", "增量配置文件");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    YAML::Node config       = YAML::LoadFile(FLAGS_config_file);
    std::string bag_path    = config["main"]["bag_path"].as<std::string>();  // 数据集
    std::string laser_topic = config["main"]["laser_topic"].as<std::string>();
    std::string imu_topic   = config["main"]["imu_topic"].as<std::string>();
    std::string odom_topic  = config["main"]["odom_topic"].as<std::string>();
    int with_imu            = config["main"]["with_imu"].as<int>();  // 0: LO | 1:eskf LIO | 2:ieskf LIO
    bool with_display       = config["main"]["with_display"].as<bool>();
    bool with_loopClosure   = config["main"]["with_loopClosure"].as<bool>();
    bool save_map           = config["main"]["save_map"].as<bool>();

    LOG(INFO) << "laser topic: " << laser_topic;
    LOG(INFO) << "imu topic: " << imu_topic;
    LOG(INFO) << "odom topic: " << odom_topic;

    auto NdtLIO = std::make_shared<sad::IncrementalNDTLO>( FLAGS_config_file, with_imu, with_display, with_loopClosure);
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NDTLIONode>("ndt_lio_node", NdtLIO, imu_topic, laser_topic, odom_topic, with_imu);
    rclcpp::spin(node);

    if ( save_map ){
        std::string fileName = "../map/global_map.png";
        cv::imwrite(fileName, NdtLIO->getGlobalMap(2000));
        LOG(INFO) << "map is saved in: " << fileName;
    }

    rclcpp::shutdown();
    return 0;
}






