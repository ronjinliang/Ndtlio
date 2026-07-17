#include "../include/occupancy_map.h"
#include "../common/math_utils.h"

#include <glog/logging.h>
#include <execution>
#include <yaml-cpp/yaml.h>

namespace sad {

void OccupancyMap::addLidarFrame(std::shared_ptr<Frame> frame ){
    auto & pts = frame->pts_;

    // 先计算末端点所在网格
    std::set<Vec2i, less_vec<2>> endpoints;

    for ( size_t i = 0; i < pts.size(); ++i ) {
        Vec2i img_point = world2Image(frame->pose_ * pts[i]);  /// 转换到图像坐标系
        // 检查是否越界
        if (img_point[0] < 0) outFlags_.left_outside_ = true;
        if (img_point[1] < 0) outFlags_.top_outside_ = true;
        if (img_point[0] >= occupancy_grid_.cols) outFlags_.right_outside_ = true;
        if (img_point[1] >= occupancy_grid_.rows) outFlags_.bottom_outside_ = true;
        endpoints.emplace(img_point);
    }

    Vec2i start = world2Image(frame->pose_.translation());
    /// 涂白
    std::for_each(std::execution::par_unseq, endpoints.begin(), endpoints.end(),
                [this, &start](const auto & pt) { bresenhamFilling(start, pt); });
    /// 末端点涂黑
    std::for_each(endpoints.begin(), endpoints.end(), [this](const auto & pt) { setPoint(pt, true); });

    // 处理完之后再扩充, 不然会有很奇怪的bug
    std::unique_lock<std::mutex> lock(data_mutex_);
    dynamicExpand();   // 有可能扩展的同时 display 在调用 getOccupancyGridBlackWhite 这时可能导致内存访问错误
    lock.unlock();
}

void OccupancyMap::bresenhamFilling( const Vec2i & p1, const Vec2i & p2 ){
    int dx = p2.x() - p1.x();   int dy = p2.y() - p1.y();
    int ux = dx > 0 ? 1 : -1;   int uy = dy > 0 ? 1 : -1; // 方向
    dx = abs(dx);               dy = abs(dy);
    int x = p1.x();             int y = p1.y();
    
    if ( dx > dy ) {
        // 以x为增量
        int e = -dx;
        for (int i = 0; i < dx; ++i) {
            x += ux;
            e += 2 * dy;
            if (e >= 0) {
                y += uy;
                e -= 2 * dx;
            }
            if (Vec2i(x, y) != p2) setPoint(Vec2i(x, y), false);
        }
    } else {
        int e = -dy;
        for (int i = 0; i < dy; ++i) {
            y += uy;
            e += 2 * dx;
            if (e >= 0) {
                x += ux;
                e -= 2 * dy;
            }
            if (Vec2i(x, y) != p2)  setPoint(Vec2i(x, y), false);
        }
    }
}

void OccupancyMap::dynamicExpand(){
    bool need_expand = outFlags_.left_outside_ || outFlags_.top_outside_ || outFlags_.right_outside_ || outFlags_.bottom_outside_;
    if (!need_expand) return;

    int top_boarder = 0, bottom_boarder = 0, left_boarder = 0, right_boarder = 0;
    if ( outFlags_.left_outside_ ){
        left_boarder = opts_.image_size_ / 5;
        center_image_.x() = center_image_.x() + left_boarder;
    }
    
    if ( outFlags_.top_outside_ ){
        top_boarder = opts_.image_size_ / 5;
        center_image_.y() = center_image_.y() + top_boarder;
    }
    
    if ( outFlags_.right_outside_ ){
        right_boarder = opts_.image_size_ / 5;
    }
    
    if ( outFlags_.bottom_outside_ ){
        bottom_boarder = opts_.image_size_ / 5;
    }

    // cv::Mat使用引用计数，所以赋值操作是浅拷贝，不会复制数据。
    // 因此，可以直接使用赋值操作 occupancy_grid_ = occupancy_grid_copy;，
    // 这样两个矩阵会共享数据，直到其中一个被修改时才会进行深拷贝（写时复制）。
    
    // 执行扩展
    cv::Mat expanded_grid;
    try {
        cv::copyMakeBorder(occupancy_grid_, expanded_grid,
            top_boarder, bottom_boarder, left_boarder, right_boarder,
            cv::BORDER_CONSTANT, cv::Scalar(127));
        
        occupancy_grid_ = expanded_grid;
        std::cout << "Grid expanded to: " 
                  << occupancy_grid_.cols << "x" << occupancy_grid_.rows 
                  << " (added: L:" << left_boarder << " R:" << right_boarder
                  << " T:" << top_boarder << " B:" << bottom_boarder << ")"
                  << ", center: " << center_image_.transpose() << std::endl;
        
    } catch (const cv::Exception& e) {
        std::cerr << "Failed to expand grid: " << e.what() << std::endl;
    }
    outFlags_ = OutsideFlags();
}

void OccupancyMap::setPoint( const Vec2i & pt, bool occupy ){
    int x = pt[0], y = pt[1];

    /// 有无 occupied 都要检查，不然会报内存访问错误
    if ( x < 0 ) outFlags_.left_outside_ = true;
    if ( y < 0 ) outFlags_.top_outside_  = true;
    if ( x >= occupancy_grid_.cols ) outFlags_.right_outside_ = true;
    if ( y >= occupancy_grid_.rows ) outFlags_.bottom_outside_ = true;
    if (outFlags_.left_outside_ || outFlags_.top_outside_ || outFlags_.right_outside_ || outFlags_.bottom_outside_ ) return;

    /// 设置上下限
    uchar value = occupancy_grid_.at<uchar>(y,x);
    if ( occupy ) {
        if ( value > 117 ) occupancy_grid_.ptr<uchar>(y)[x] -= 1;
    } else {
        if ( value < 137 ) occupancy_grid_.ptr<uchar>(y)[x] += 1;
    }
}


/************************************************* 显示地图 ***************************************/
/************************************************* 显示地图 ***************************************/
/************************************************* 显示地图 ***************************************/

/// 获取黑白灰形式的占据栅格，作可视化使用
cv::Mat OccupancyMap::getOccupancyGridBlackWhite() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    // 检查网格是否为空
    if (occupancy_grid_.empty()) return cv::Mat(occupancy_grid_.rows, occupancy_grid_.cols, CV_8UC3);

    cv::Mat image(occupancy_grid_.rows, occupancy_grid_.cols, CV_8UC3);
    for (int y = 0; y < occupancy_grid_.rows; ++y) {  // row 列
        const uchar* grid_row = occupancy_grid_.ptr<uchar>( occupancy_grid_.rows - y - 1);   // 转换y的方向去显示
        cv::Vec3b*  image_row = image.ptr<cv::Vec3b>(y);
        for (int x = 0; x < occupancy_grid_.cols; ++x) {  // column 行
            uchar value = grid_row[x];
            if (value == 127) {
                image_row[x] = cv::Vec3b(127, 127, 127);
            } else if (value < 127) {
                image_row[x] = cv::Vec3b(0, 0, 0);
            } else { // value > 127
                image_row[x] = cv::Vec3b(255, 255, 255);
            }
        }
    }
    return image;
}



}
