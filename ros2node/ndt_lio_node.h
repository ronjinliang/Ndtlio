#ifndef __NDT_LIO_NODE_H
#define __NDT_LIO_NODE_H

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <origincar_msg/msg/data.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include "2dNdtLIO/include/incrementalNDTLO.h"

/* laser gyro_link */

class NDTLIONode : public rclcpp::Node {
public:
    NDTLIONode( const std::string & nodeName, std::shared_ptr<sad::IncrementalNDTLO> ndt_lio,
        const std::string & imu_topic, const std::string & scan_topic, const std::string & odom_topic,
        int with_imu );
    ~NDTLIONode();
        
    void close();
    
private:

    void pubOccupancyMapThread();
    // 最好的处理方式是新建一个进程来处理, callback缓冲数据就行
    void imuCallback( const sensor_msgs::msg::Imu::SharedPtr msg );
    void scanCallback( const sensor_msgs::msg::LaserScan::SharedPtr msg );
    void odomCallback( const origincar_msg::msg::Data::SharedPtr msg );

    double timeStamp( const int & sec, const int & nanosec );
    void publishOccupancyMap();
    void publishGlobalPointCloud();
    void publishMatchPointCloud();
    void updateAndPublishTrajectory( const SE2 & pose );
    void publichBaseToMap( const SE2 & pose );
    void publishStaticTransform();

private:
    std::mutex data_mutex_;
    std::thread pub_occupancyMap_thread;
    std::atomic<bool> thread_running_ = true;

    int with_imu_ = 1;  // 0:LO | 1:eskf | 2:ieskf
    bool imu_init_success_ = false;

    std::shared_ptr<sad::IncrementalNDTLO> ndt_lio_ = nullptr;

    nav_msgs::msg::Path trajectory_;      // 轨迹
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occu_pub_ = nullptr;  // 占据图

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pcl_pub_ = nullptr;   // 全局点云地图

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = nullptr;         // 订阅 imu 数据
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_ = nullptr;  // 订阅雷达数据
    rclcpp::Subscription<origincar_msg::msg::Data>::SharedPtr odom_sub_ = nullptr;     // 订阅轮速计数据

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_laser_to_base_broadcaster_ = nullptr;  // laser to base 的静态变换
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_imu_to_base_broadcaster_ = nullptr;    // imu to base 的静态变换
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_base_to_map_broadcaster_ = nullptr;          // base to map 的动态变换

    rclcpp::TimerBase::SharedPtr timer_static_tf_ = nullptr;  // 用于 2 s定时发布静态变换
};


#endif // __NDT_LIO_NODE_H
