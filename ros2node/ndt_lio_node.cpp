#include "2dNdtLIO/ros2node/ndt_lio_node.h"
#include "2dNdtLIO/include/frame.h"
#include "2dNdtLIO/common/imu.h"
#include "2dNdtLIO/common/odom.h"


NDTLIONode::NDTLIONode( const std::string & nodeName, std::shared_ptr<sad::IncrementalNDTLO> ndt_lio,
    const std::string & imu_topic, const std::string & scan_topic, const std::string & odom_topic,
    int with_imu)
    : Node(nodeName), ndt_lio_(ndt_lio), with_imu_(with_imu) {
    
    trajectory_.header.frame_id = "map";
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory", rclcpp::QoS(10));
    occu_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(10));

    global_pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_point_cloud", rclcpp::QoS(10));   // 全局点云地图

    // rclcpp::QoS(1) 历史深度为1
    if (with_imu_ != 0) {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::QoS(1),
            std::bind(&NDTLIONode::imuCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<origincar_msg::msg::Data>(odom_topic, rclcpp::QoS(1),
            std::bind(&NDTLIONode::odomCallback, this, std::placeholders::_1));
    }

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::QoS(1),
        std::bind(&NDTLIONode::scanCallback, this, std::placeholders::_1));
        
    
    tf_laser_to_base_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_imu_to_base_broadcaster_   = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_base_to_map_broadcaster_   = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 定时 2 秒发布一次静态坐标
    timer_static_tf_ = this->create_wall_timer(
        std::chrono::seconds(2),std::bind(&NDTLIONode::publishStaticTransform, this));

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
        // 1.占据图
        publishOccupancyMap();

        // 2.全局体素
        publishGlobalPointCloud();
    }
}

void NDTLIONode::imuCallback( const sensor_msgs::msg::Imu::SharedPtr msg ){
    IMUPtr imu      = std::make_shared<sad::IMU>();
    imu->timestamp_ = timeStamp(msg->header.stamp.sec, msg->header.stamp.nanosec);
    imu->acce_      = Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    imu->gyro_      = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    
    if ( imu_init_success_ ) {
        ndt_lio_->processIMU(imu);
        auto current_frame = ndt_lio_->getFrontend()->getCurrentFrame();
        if ( current_frame ) {  // 更新 base to map
            SE2 baseToMap = current_frame->pose_;
            publichBaseToMap( baseToMap );
        }
    } else {
        imu_init_success_ = ndt_lio_->initIMU(imu);  // 先初始化 imu
    }

}

void NDTLIONode::scanCallback( const sensor_msgs::msg::LaserScan::SharedPtr msg ){
    // LIO
    if ( imu_init_success_ ) {
        ndt_lio_->processScan(msg);
        auto frontend = ndt_lio_->getFrontend();

        if ( frontend->isKeyframe() ) {  // 是关键帧就更新并发布轨迹
            SE2 baseToMap = frontend->getCurrentFrame()->pose_;
            updateAndPublishTrajectory( baseToMap );
        }
    }

    // LO 的话, 那就由雷达发布 base to map
    if ( with_imu_ == 0 ) {
        ndt_lio_->processScan(msg);
        // publish base to map
        auto frontend = ndt_lio_->getFrontend();
        SE2 baseToMap = frontend->getCurrentFrame()->pose_;
        publichBaseToMap( baseToMap );

        if ( frontend->isKeyframe() ) {  // 是关键帧就更新并发布轨迹
            updateAndPublishTrajectory( baseToMap );
        }
    }

}

void NDTLIONode::odomCallback( const origincar_msg::msg::Data::SharedPtr msg ){
    if ( imu_init_success_ ) {
        std::shared_ptr<sad::Odom> odom = std::make_shared<sad::Odom>(msg->x);
        ndt_lio_->proccessOdom(odom);
    }
}

void NDTLIONode::publishOccupancyMap(){
    cv::Mat global_map = ndt_lio_->getMap()->getOccupancyMap().getOccupancyGrid();
    nav_msgs::msg::OccupancyGrid occu_grid;
    occu_grid.header.frame_id = "map";
    occu_grid.header.stamp = this->now();
    occu_grid.info.width  = global_map.cols;  // x
    occu_grid.info.height = global_map.rows;  // y
    occu_grid.info.resolution = 1.0 / ndt_lio_->getMap()->getOccupancyMap().getResolution();
    Vec2d center = ndt_lio_->getMap()->getOccupancyMap().getCenter();
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

void NDTLIONode::publishGlobalPointCloud(){
    std::unique_lock<std::mutex> lock(data_mutex_);
    std::vector<Vec2d> voxels = ndt_lio_->getMap()->getNdt().getVoxels();  // 深拷贝所有体素
    lock.unlock();
    sensor_msgs::msg::PointCloud2 global_pcl;
    global_pcl.header.frame_id = "map";  // 设置头部
    global_pcl.header.stamp = this->now();
    // 设置固定结构
    global_pcl.height = 1;             // 无序点云设为1
    global_pcl.width = voxels.size();  // 点的数量
    if ( global_pcl.width > 0 ) {
        global_pcl.is_dense = true;
        global_pcl.is_bigendian = false;
        // 定义字段：2D点云只需要x,y，但可以添加强度等
        sensor_msgs::PointCloud2Modifier modifier(global_pcl);
        modifier.setPointCloud2Fields(3, // 字段数量
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        // 填充数据
        sensor_msgs::PointCloud2Iterator<float> iter_x(global_pcl, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(global_pcl, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(global_pcl, "z");
        for ( auto & v : voxels ) {
            *iter_x = v(0);    *iter_y = v(1);    *iter_z = 0.0f;
            ++iter_x;          ++iter_y;          ++iter_z;
        }
        global_pcl_pub_->publish(global_pcl);
    }
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
    transform_stamped.header.frame_id = "map";
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
    tf_laser_to_base_broadcaster_->sendTransform(transform_stamped);
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
    tf_laser_to_base_broadcaster_->sendTransform(transform_stamped);
    
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
    tf_imu_to_base_broadcaster_->sendTransform(transform_stamped);
}

