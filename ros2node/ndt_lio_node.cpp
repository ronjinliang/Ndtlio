#include "../ros2node/ndt_lio_node.h"
#include "../include/frame.h"
#include "../common/imu.h"
#include "../common/odom.h"
#include "std_msgs/msg/int32.hpp"

#include <yaml-cpp/yaml.h>

NDTLIONode::NDTLIONode( const std::string & nodeName ) : Node(nodeName) {
    // 读取配置参数;
    this->declare_parameter<std::string>("config_file", "/home/lrj/origincar_ws/src/Ndtlio/config/mapping.yaml");
    this->get_parameter("config_file", config_file_);
    LOG(INFO) << "use config: " << config_file_;
    
    YAML::Node config       = YAML::LoadFile(config_file_);
    std::string laser_topic = config["main"]["laser_topic"].as<std::string>();
    std::string imu_topic   = config["main"]["imu_topic"].as<std::string>();
    std::string odom_topic  = config["main"]["odom_topic"].as<std::string>();
    with_imu_               = config["main"]["with_imu"].as<int>();  // 0: LO | 1:eskf LIO | 2:ieskf LIO
    localization_mode_      = config["main"]["localization_mode"].as<bool>();

    LOG(INFO) << "laser topic: " << laser_topic;
    LOG(INFO) << "imu topic: " << imu_topic;
    LOG(INFO) << "odom topic: " << odom_topic;

    // 构建lio对象
    ndt_lio_ = sad::IncrementalNDTLO(config_file_, with_imu_);

    trajectory_.header.frame_id = ref_frame_;
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory", rclcpp::QoS(10));
    scan_plc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan_point_cloud", rclcpp::QoS(10));    // 雷达点云
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ndt_odom", rclcpp::QoS(10));

    if ( localization_mode_ ) {
        LOG(INFO) << "localization mode";
        ref_frame_ = "odom";;

        initpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", rclcpp::QoS(10),
            std::bind(&NDTLIONode::initposeCallback, this, std::placeholders::_1));
    } else {
        LOG(INFO) << "slam mode";
        ref_frame_ = "map";
        
        // 一定要跟  ros2 run nav2_map_server map_saver_cli -t map -f origincar_map 
        // 这里面的 QoS 对应上，否则不能保存保存地图
        rclcpp::QoS map_qos(1);
        map_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        map_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        occu_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
    }

    if (with_imu_ != 0) {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::QoS(100),
            std::bind(&NDTLIONode::imuCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<origincar_msg::msg::Data>(odom_topic, rclcpp::QoS(100),
            std::bind(&NDTLIONode::odomCallback, this, std::placeholders::_1));
    }

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_topic, rclcpp::QoS(1),
        std::bind(&NDTLIONode::scanCallback, this, std::placeholders::_1));
    sign_sub_ = this->create_subscription<std_msgs::msg::Int32>("/sign4return", rclcpp::QoS(10),
        std::bind(&NDTLIONode::signCallback, this, std::placeholders::_1));

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_broadcaster_        = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 定时 2 秒发布一次静态坐标
    timer_static_tf_ = this->create_wall_timer(
        std::chrono::seconds(2),std::bind(&NDTLIONode::publishStaticTransform, this));
    
    if (localization_mode_ == true) {
        timer_tf_ = this->create_wall_timer(
            std::chrono::milliseconds(10),std::bind(&NDTLIONode::pubPoseTimerCallback, this));
    }

    pub_occupancyMap_thread = std::thread(std::bind(&NDTLIONode::pubOccupancyMapThread, this));
}

NDTLIONode::~NDTLIONode(){
    thread_running_.store(false);
    if ( pub_occupancyMap_thread.joinable() ) {
        pub_occupancyMap_thread.join();
    }
}

void NDTLIONode::pubOccupancyMapThread(){
    while (thread_running_.load()) {
        usleep(100000);  // 10Hz
        // 1.占据图  定位模式
        if ( localization_mode_ == false ) {
            std::lock_guard<std::mutex> lock(mtx_);
            publishOccupancyMap();
        }
    }
}

void NDTLIONode::signCallback( const std_msgs::msg::Int32::SharedPtr msg ){
    int sign = msg->data;
    if (sign == -2){
        std::lock_guard<std::mutex> lock(mtx_);
        ndt_lio_ = sad::IncrementalNDTLO(config_file_, with_imu_);
        imu_init_success_ = false;
        has_initial_pose_ = false;
        trajectory_.poses.clear();
        LOG(INFO) << "reset lio";
    }
}

void NDTLIONode::pubPoseTimerCallback(){
    if ( has_initial_pose_ == false ) return;
    // px, py, vx, vy, theta, bg, bax, bay  8d getNominal getDtheta
    
    Vec8d state = ndt_lio_.getESKF()->getNominal();

    odom_.header.stamp = this->now();
    odom_.pose.pose.position.x = state[0];
    odom_.pose.pose.position.y = state[1];
    odom_.twist.twist.linear.x = state[2];
    odom_.twist.twist.linear.y = state[3];
    odom_.pose.pose.orientation.x = 0.0;
    odom_.pose.pose.orientation.y = 0.0;
    odom_.pose.pose.orientation.z = std::sin(state[4] / 2.0);
    odom_.pose.pose.orientation.w = std::cos(state[4] / 2.0);
    odom_.twist.twist.angular.z = ndt_lio_.getESKF()->getDtheta();

    odom_pub_->publish(odom_);
}


void NDTLIONode::initposeCallback( const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg ){
    // yaw = 2 * atan2(z, w)
    T_map_odom_.pose = msg->pose;
    has_initial_pose_ = true;
    LOG(INFO) << "initial pose, odom -> map";
}

void NDTLIONode::imuCallback( const sensor_msgs::msg::Imu::SharedPtr msg ){
    if ( localization_mode_ == true && has_initial_pose_ == false ) return;

    IMUPtr imu      = std::make_shared<sad::IMU>();
    imu->timestamp_ = timeStamp(msg->header.stamp.sec, msg->header.stamp.nanosec);
    imu->acce_      = Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    imu->gyro_      = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    
    if ( imu_init_success_ ) {
        ndt_lio_.processIMU(imu);
        auto current_frame = ndt_lio_.getFrontend()->getCurrentFrame();
        if ( current_frame ) {  // 更新 base to odom
            SE2 baseToMap = current_frame->pose_;
            publichBaseToMap( baseToMap );
        }
    } else {
        imu_init_success_ = ndt_lio_.initIMU(imu);  // 先初始化 imu
    }

}

void NDTLIONode::scanCallback( const sensor_msgs::msg::LaserScan::SharedPtr msg ){
    if ( localization_mode_ == true && has_initial_pose_ == false ) return;

    // LIO
    if ( imu_init_success_ ) {
        ndt_lio_.processScan(msg);
        auto frontend = ndt_lio_.getFrontend();
        auto frame = frontend->getCurrentFrame();

        SE2 baseToMap = frame->pose_;
        publichBaseToMap( baseToMap );

        sensor_msgs::msg::PointCloud2 laser_scan_pcl;
        laser_scan_pcl.header.frame_id = msg->header.frame_id;  // 设置头部
        laser_scan_pcl.header.stamp = msg->header.stamp;
        // 设置固定结构
        laser_scan_pcl.height = 1;             // 无序点云设为1
        laser_scan_pcl.width = frame->pts_.size();  // 点的数量
        if ( laser_scan_pcl.width > 0 ) {
            laser_scan_pcl.is_dense = true;
            laser_scan_pcl.is_bigendian = false;
            // 定义字段：2D点云只需要x,y，但可以添加强度等
            sensor_msgs::PointCloud2Modifier modifier(laser_scan_pcl);
            modifier.setPointCloud2Fields(3, // 字段数量
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32);
            // 填充数据
            sensor_msgs::PointCloud2Iterator<float> iter_x(laser_scan_pcl, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(laser_scan_pcl, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(laser_scan_pcl, "z");
            for ( auto & pt : frame->pts_ ) {
                *iter_x = pt(0);    *iter_y = pt(1);    *iter_z = 0.0;
                ++iter_x;           ++iter_y;           ++iter_z;
            }
            scan_plc_pub_->publish(laser_scan_pcl);
        }

        if ( frontend->isKeyframe() ) {  // 是关键帧就更新并发布轨迹
            SE2 baseToMap = frontend->getCurrentFrame()->pose_;
            updateAndPublishTrajectory( baseToMap );
        }
    }

    // LO 的话, 那就由雷达发布 base to map
    if ( with_imu_ == 0 ) {
        ndt_lio_.processScan(msg);
        // publish base to map
        auto frontend = ndt_lio_.getFrontend();
        SE2 baseToMap = frontend->getCurrentFrame()->pose_;
        publichBaseToMap( baseToMap );

        if ( frontend->isKeyframe() ) {  // 是关键帧就更新并发布轨迹
            updateAndPublishTrajectory( baseToMap );
        }
    }

}

void NDTLIONode::odomCallback( const origincar_msg::msg::Data::SharedPtr msg ){
    if ( localization_mode_ == true && has_initial_pose_ == false ) return;

    if ( imu_init_success_ ) {
        std::shared_ptr<sad::Odom> odom = std::make_shared<sad::Odom>(msg->x);
        ndt_lio_.proccessOdom(odom);
    }
}

void NDTLIONode::publishOccupancyMap(){
    cv::Mat global_map = ndt_lio_.getMap()->getOccupancyMap().getOccupancyGrid();
    nav_msgs::msg::OccupancyGrid occu_grid;
    occu_grid.header.frame_id = ref_frame_;
    occu_grid.header.stamp = this->now();
    occu_grid.info.width  = global_map.cols;  // x
    occu_grid.info.height = global_map.rows;  // y
    occu_grid.info.resolution = 1.0 / ndt_lio_.getMap()->getOccupancyMap().getResolution();
    Vec2d center = ndt_lio_.getMap()->getOccupancyMap().getCenter();
    occu_grid.info.origin.position.x = - center.x() * occu_grid.info.resolution;
    occu_grid.info.origin.position.y = - center.y() * occu_grid.info.resolution;
    occu_grid.info.origin.position.z = 0.0;
    occu_grid.info.origin.orientation.x = 0.0;
    occu_grid.info.origin.orientation.y = 0.0;
    occu_grid.info.origin.orientation.z = 0.0;
    occu_grid.info.origin.orientation.w = 1.0;

    occu_grid.data.resize(global_map.cols * global_map.rows);
    int y_idx = 0;
    for ( int y = 0; y < global_map.rows; ++y ) {
        const uchar * grid_row = global_map.ptr<uchar>( y );
        for ( int x = 0; x < global_map.cols; ++ x ) {
            int idx = x + y_idx ; // x + y * global_map.cols
            uchar value = grid_row[x];
            if ( value == 127 )     occu_grid.data[idx] = -1;   // 未知  == 127
            else if ( value < 127 ) occu_grid.data[idx] = 100;  // 占用  <  127
            else                    occu_grid.data[idx] = 0;    // 空闲  >  127
        }
        y_idx += global_map.cols;
    }
    occu_pub_->publish(occu_grid);
}

double NDTLIONode::timeStamp( const int & sec, const int & nanosec ){
    return double(sec) + double(nanosec) * 1e-9;
}

void NDTLIONode::updateAndPublishTrajectory( const SE2 & baseToMap ){
    trajectory_.header.stamp = this->now();
    geometry_msgs::msg::PoseStamped pose;
    pose.header = trajectory_.header;
    pose.pose.position.x = baseToMap.translation().x();
    pose.pose.position.y = baseToMap.translation().y();
    pose.pose.position.z = 0.0;
    double theta = baseToMap.so2().log();
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(theta * 0.5);
    pose.pose.orientation.w = std::cos(theta * 0.5);
    trajectory_.poses.push_back(pose);
    path_pub_->publish(trajectory_);
}

void NDTLIONode::publichBaseToMap( const SE2 & pose ){
    auto transform_stamped = geometry_msgs::msg::TransformStamped();

    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = ref_frame_;
    transform_stamped.child_frame_id = "base";
    // translation
    transform_stamped.transform.translation.x = pose.translation().x();
    transform_stamped.transform.translation.y = pose.translation().y();
    transform_stamped.transform.translation.z = 0.0;
    // rotation
    double theta = pose.so2().log();
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = std::sin(theta * 0.5);
    transform_stamped.transform.rotation.w = std::cos(theta * 0.5);
    // publish
    tf_broadcaster_->sendTransform(transform_stamped);
}

void NDTLIONode::publishStaticTransform(){
    auto transform_stamped = geometry_msgs::msg::TransformStamped();

    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "base";
    transform_stamped.child_frame_id = "laser";
    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;
    tf_static_broadcaster_->sendTransform(transform_stamped);
    
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "base";
    transform_stamped.child_frame_id = "imu";
    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;
    tf_static_broadcaster_->sendTransform(transform_stamped);

    if ( localization_mode_ ) {
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "odom";
        transform_stamped.transform.translation.x = T_map_odom_.pose.pose.position.x;
        transform_stamped.transform.translation.y = T_map_odom_.pose.pose.position.y;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = T_map_odom_.pose.pose.orientation.x;
        transform_stamped.transform.rotation.y = T_map_odom_.pose.pose.orientation.y;
        transform_stamped.transform.rotation.z = T_map_odom_.pose.pose.orientation.z;
        transform_stamped.transform.rotation.w = T_map_odom_.pose.pose.orientation.w;
        tf_static_broadcaster_->sendTransform(transform_stamped);
    }
}

