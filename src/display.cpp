#include "2dNdtLIO/include/display.h"

#include <execution>
#include <opencv2/opencv.hpp>

namespace sad {

/////// NOTE: 图像坐标的 y 轴与实际定义的方向相反
Display::Display( const int max_size ) : max_size_(max_size) {
    display_running_.store(true);
    // 创建显示进程
    display_thread_ = std::thread(std::bind(&Display::displayLoop, this));
}

void Display::close(){
    display_running_.store(false);
    display_thread_.join();
    LOG(INFO) << "Stop display.";
}


void Display::updateCurrentFrame( std::shared_ptr<Frame> current_frame ) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    current_frame_ = current_frame;
}

const cv::Mat Display::getGlobalMap( int max_size ){
    // 可视化
    auto & occu_map = map_->getOccupancyMap();
    cv::Mat occu_image = occu_map.getOccupancyGridBlackWhite();
    float resolution = occu_map.getResolution();
    Vec2d center = occu_map.getCenter();

    // 如果图像太大就缩小显示
    cv::Mat display_image;
    double scale = 1.0;
    if ( occu_image.cols > max_size || occu_image.rows > max_size ) {
        scale = std::min(
            static_cast<double>( max_size ) / occu_image.cols,
            static_cast<double>( max_size ) / occu_image.rows);
        cv::resize(occu_image, display_image, cv::Size(), scale, scale);
    } else {
        display_image = occu_image;
    }

    center = center * scale;
    resolution = resolution * scale;

    // 显示轨迹 TUDO 加上这个画出轨迹之后会存在一点问题
    auto frames = map_->getAllFramesByCopy();
    for ( const auto & frame : frames ) {
        Vec2d p_map = frame.second->pose_.translation() * resolution + center;
        cv::circle(display_image, cv::Point2d(p_map.x(), display_image.rows - p_map.y()), 1, cv::Scalar(0, 0, 255), 1);
    }

    // 画出体素
    auto voxels = map_->getNdt().getVoxels();  // 深拷贝所有体素
    for ( auto voxel : voxels ) {
        Vec2d p_map = Vec2d( voxel(0), voxel(1) ) * resolution + center;
        cv::circle(display_image, cv::Point2f( p_map.x(), display_image.rows - p_map.y() ), 1, cv::Scalar(255, 255, 0), 1);
    }

    // 画出激光数据
    if ( current_frame_ ) visualize2DScan(current_frame_->pts_, current_frame_->pose_, display_image, center, resolution );

    // 关键帧的数量
    cv::putText(display_image, "kf: " + std::to_string(frames.size()), cv::Point(20, 30), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));
    return display_image;
}


void Display::displayLoop(){
    while (display_running_.load()) {
        usleep(33000);  // 33ms -> 30fps

        if ( current_frame_ == nullptr ) continue;

        cv::imshow("occupancy map", getGlobalMap(max_size_));
        cv::waitKey(6);
    }
}


void Display::visualize2DScan(std::vector<Vec2d> & pts, const SE2& pose, cv::Mat& image, const Vec2d & center, float resolution ) {
    if (image.data == nullptr) return;
    // 并行画出激光数据
    std::for_each(std::execution::par_unseq, pts.begin(), pts.end(),
        [&](const Vec2d & pt){
            Vec2d world_point = pose * pt;
            
            double x = world_point[0] * resolution + center[0];
            double y = world_point[1] * resolution + center[1];
            int image_x = int( x );
            int image_y = image.rows - int( y );
            if (image_x >= 0 && image_x < image.cols && image_y >= 0 && image_y < image.rows) {
                image.at<cv::Vec3b>( image_y, image_x) = cv::Vec3b(0,255,0);  // 绿色 BGR
            }
        });

    // 机器人位置
    Vec2d robot_pos = pose.translation();
    int robot_x, robot_y;
    robot_x = static_cast<int>( ( robot_pos.x() * resolution + center[0] ) );
    robot_y = static_cast<int>( ( robot_pos.y() * resolution + center[1] ) );
    if (robot_x >= 0 && robot_x < image.cols && robot_y >= 0 && robot_y < image.rows) {
        cv::circle(image, cv::Point(robot_x, image.rows - robot_y), 2, cv::Scalar(), 2);
    }
}

}
