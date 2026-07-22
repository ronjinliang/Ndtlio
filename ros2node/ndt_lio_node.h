#ifndef __NDT_LIO_NODE_H
#define __NDT_LIO_NODE_H

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <origincar_msg/msg/data.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "../include/incrementalNDTLO.h"

/* laser gyro_link */

class NDTLIONode : public rclcpp::Node {
public:
    NDTLIONode( const std::string & nodeName );
    ~NDTLIONode();
private:
    void pubOccupancyMapThread();
    void pubPoseTimerCallback();
    void initposeCallback( const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg );
    // 最好的处理方式是新建一个进程来处理, callback缓冲数据就行
    void imuCallback( const sensor_msgs::msg::Imu::SharedPtr msg );
    void scanCallback( const sensor_msgs::msg::LaserScan::SharedPtr msg );
    void odomCallback( const origincar_msg::msg::Data::SharedPtr msg );
    void signCallback( const std_msgs::msg::Int32::SharedPtr msg );

    double timeStamp( const int & sec, const int & nanosec );
    void publishOccupancyMap();
    void updateAndPublishTrajectory( const SE2 & pose );
    void publichBaseToMap( const SE2 & pose );
    void publishStaticTransform();
    void publishTransform();

    void resetOdom();
    

private:
    std::string config_file_;
    

    std::thread pub_occupancyMap_thread;
    std::atomic<bool> thread_running_ = true;

    int with_imu_ = 1;  // 0:LO | 1:eskf | 2:ieskf
    bool imu_init_success_ = false;

    bool localization_mode_ = false;
    bool has_initial_pose_ = false;
    geometry_msgs::msg::PoseWithCovarianceStamped T_map_odom_;

    std::string ref_frame_ = "map";

    sad::IncrementalNDTLO ndt_lio_;

    nav_msgs::msg::Path trajectory_;      // 轨迹
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occu_pub_ = nullptr;  // 占据图
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_plc_pub_ = nullptr;    // 雷达点云
    nav_msgs::msg::Odometry odom_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_ = nullptr;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = nullptr;         // 订阅 imu 数据
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_ = nullptr;  // 订阅雷达数据
    rclcpp::Subscription<origincar_msg::msg::Data>::SharedPtr odom_sub_ = nullptr;     // 订阅轮速计数据
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sign_sub_ = nullptr;           

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_ = nullptr;  // 静态变换
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;               // 动态变换
    

    rclcpp::TimerBase::SharedPtr timer_static_tf_ = nullptr;  // 用于 2 s定时发布静态变换
    rclcpp::TimerBase::SharedPtr timer_tf_ = nullptr;
    
    std::mutex mtx_;
};


#endif // __NDT_LIO_NODE_H
