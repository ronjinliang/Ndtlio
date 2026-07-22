#include <yaml-cpp/yaml.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "2dNdtLIO/include/incrementalNDTLO.h"
#include "2dNdtLIO/common/io_utils.h"

DEFINE_string(config_file, "/home/lrj/lidar_slam/src/2dNdtLIO/config/ndt_params_floor.yaml", "增量 ndt 配置文件");
// DEFINE_string(config_file, "/home/lrj/lidar_slam/src/2dNdtLIO/config/ndt_params_data3.yaml", "增量 ndt 配置文件");

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
    int delay_time          = config["main"]["delay_time"].as<int>() * 1000;

    LOG(INFO) << "laser topic: " << laser_topic;
    LOG(INFO) << "imu topic: " << imu_topic;
    LOG(INFO) << "odom topic: " << odom_topic;

    auto NdtLIO = std::make_shared<sad::IncrementalNDTLO>( FLAGS_config_file, with_imu, with_display, with_loopClosure );
    
    bool imu_init_success = false;
    sad::RosbagIO rosbag_io(bag_path);

    if ( laser_topic == "horizontal_laser_2d" ) {
        if ( with_imu != 0 ) {
            LOG(INFO) << "NDT LIO start.";
            rosbag_io.AddMultiScanHandle(laser_topic,
                [&](Scan2d::Ptr scan) {
                    usleep(delay_time);
                    return NdtLIO->processScan(scan);
                }
            ).AddImuHandle(imu_topic,
                [&](IMUPtr imu){
                    if ( !imu_init_success ) {  // 如果直接跳过所有数据，说明可能是 imu_init_success 没有设置1
                        imu_init_success = NdtLIO->initIMU(imu);
                        return imu_init_success;
                    }
                    return NdtLIO->processIMU(imu);
                }
            ).Go();
        } else {
            LOG(INFO) << "NDT LO start.";
            rosbag_io.AddMultiScanHandle(laser_topic,
                [&](Scan2d::Ptr scan) {
                    usleep(delay_time);
                    return NdtLIO->processScan(scan);
                }
            ).Go();
        }
    } else {
        if ( with_imu != 0 ) {
            LOG(INFO) << "NDT LIO start.";
            rosbag_io.AddScan2DHandle(laser_topic,
                [&](Scan2d::Ptr scan) {
                    usleep(delay_time);  // wait 100ms 非常慢
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
                    usleep(delay_time);
                    return NdtLIO->processScan(scan);
                }
            ).Go();
        }
    }


    if ( save_map ){
        std::string fileName = "../map/global_map.png";
        cv::imwrite(fileName, NdtLIO->getGlobalMap(2000));
        LOG(INFO) << "map is saved in: " << fileName;
    }

    return 0;
}






