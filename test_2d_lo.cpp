#include <yaml-cpp/yaml.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "2dNdtLIO(preintegration)/incrementalNDTLO.h"
#include "common/io_utils.h"

DEFINE_string(config_file, "/home/lrj/lidar_slam/src/2dNdtLIO(preintegration)/config/ndt_params_origincar.yaml", "增量 ndt 配置文件");

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
    bool multi_scan         = config["main"]["multi_scan"].as<bool>();
    int with_imu            = config["main"]["with_imu"].as<int>();  // 0: LO | 1:eskf LIO | 2:ieskf LIO
    bool save_map           = config["main"]["save_map"].as<bool>();

    LOG(INFO) << "laser topic: " << laser_topic;
    LOG(INFO) << "imu topic: " << imu_topic;
    LOG(INFO) << "odom topic: " << odom_topic;

    auto NdtLIO = std::make_shared<sad::IncrementalNDTLO>( FLAGS_config_file, with_imu );
    
    bool imu_init_success = false;
    sad::RosbagIO rosbag_io(bag_path, sad::DatasetType::WXB_3D);

    
    if ( multi_scan == true ) {  // cartographer 的数据集是多回波雷达
        if ( with_imu != 0 ) {
            rosbag_io.AddMultiScan2DHandle(laser_topic,
                [&](MultiScan2d::SharedPtr scan){
                    usleep(10000);
                    if ( imu_init_success ) {
                        return NdtLIO->provessMultiScan(scan);
                    }
                    return true;
                }).AddImuHandle( imu_topic,
                [&](IMUPtr imu){
                    if ( !imu_init_success ) {  // 如果直接跳过所有数据，说明可能是 imu_init_success 没有设置1
                        imu_init_success = NdtLIO->initIMU(imu);
                        return imu_init_success;
                    }
                    return NdtLIO->processIMU(imu);
                }).Go();
        } else {
            rosbag_io.AddMultiScan2DHandle(laser_topic, [&](MultiScan2d::SharedPtr scan){ return NdtLIO->provessMultiScan(scan); }).Go();
        }

    }
    
    
    
    
    else {
        if ( with_imu != 0 ) {
            LOG(INFO) << "NDT LIO start.";
            rosbag_io.AddScan2DHandle(laser_topic,
                [&](Scan2d::Ptr scan) {
                    usleep(10000);
                    if ( imu_init_success ) return NdtLIO->processScan(scan);
                    return true;
                }
            ).AddImuHandle(imu_topic,
                [&](IMUPtr imu){
                    if ( !imu_init_success ) {  // 如果直接跳过所有数据，说明可能是 imu_init_success 没有设置1
                        imu_init_success = NdtLIO->initIMU(imu);
                        return imu_init_success;
                    }
                    return NdtLIO->processIMU(imu);
                }
            ).AddOdomHandle(odom_topic,
                [&]( std::shared_ptr<sad::Odom> odom ) {
                    if ( imu_init_success ) return NdtLIO->proccessOdom(odom);
                    return true;
                }
            ).Go();
        } else {
            LOG(INFO) << "NDT LO start.";
            rosbag_io.AddScan2DHandle(laser_topic,
                [&](Scan2d::Ptr scan) {
                    usleep(10000);
                    return NdtLIO->processScan(scan);
                }
            ).Go();
        }
    }

    NdtLIO->close();

    if ( save_map ){
        cv::imwrite("../map/global_map.png", NdtLIO->getGlobalMap(2000));
    }
    return 0;
}






