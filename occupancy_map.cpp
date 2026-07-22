#include "2dNdtLIO(preintegration)/occupancy_map.h"
#include "common/math_utils.h"

#include <glog/logging.h>
#include <execution>
#include <yaml-cpp/yaml.h>

namespace sad {

/////// NOTE: 图像坐标的 y 轴与实际定义的方向相反
/// 往这个占据栅格地图中增加一个frame
void OccupancyMap::addLidarFrame(std::shared_ptr<Frame> frame, Scan2d::Ptr scan ){
    auto & pts = frame->pts_;
    SE2 pose_in_map = pose_.inverse() * frame->pose_;  // T_sw * T_wc   雷达到地图
    float theta = pose_in_map.so2().log();

    // 先计算末端点所在网格
    std::set<Vec2i, less_vec<2>> endpoints;

    for ( size_t i = 0; i < pts.size(); ++i ) {

        Vec2i img_point = world2Image(frame->pose_ * pts[i]);
        // 检查是否越界
        if (img_point[0] < 0) left_outside_ = true;
        if (img_point[1] < 0) top_outside_ = true;
        if (img_point[0] >= occupancy_grid_.cols) right_outside_ = true;
        if (img_point[1] >= occupancy_grid_.rows) bottom_outside_ = true;

        endpoints.emplace(img_point);
    }

    if ( opts_.method_ == GridMethod::MODEL_POINTS ) {
        // 遍历模板，生成白色点
        std::for_each(std::execution::par_unseq, model_.begin(), model_.end(),
            [&](const Model2DPoint& pt) {
                Vec2i pos_in_image = world2Image(frame->pose_.translation());
                Vec2i pw = pos_in_image + Vec2i(pt.dx_, pt.dy_);

                if (pt.range_ < opts_.closest_th_) {
                    // 小距离内认为无物体
                    setPoint(pw, false);
                    return;
                }

                double angle = pt.angle_ - theta;  // 激光系下角度
                double range = findRangeInAngle(angle, scan);

                if (range < scan->range_min || range > scan->range_max) {
                    /// 某方向无测量值时，认为无效
                    /// 但离机器比较近时，涂白
                    if (pt.range_ < opts_.endpoint_close_th_) {
                        setPoint(pw, false);
                    }
                    return;
                }

                if (range > pt.range_ && endpoints.find(pw) == endpoints.end()) {
                    /// 末端点与车体连线上的点，涂白
                    setPoint(pw, false);
                }
            });
    } else {
        Vec2i start = world2Image(frame->pose_.translation());
        std::for_each(std::execution::par_unseq, endpoints.begin(), endpoints.end(),
                    [this, &start](const auto & pt) { bresenhamFilling(start, pt); });
    }
    /// 末端点涂黑
    std::for_each(endpoints.begin(), endpoints.end(), [this](const auto & pt) { setPoint(pt, true); });

    std::unique_lock<std::mutex> lock(data_mutex_);
    dynamicExpand();   // 有可能扩展的同时 display 在调用 getOccupancyGridBlackWhite 这时可能导致内存访问错误
    lock.unlock();
}

void OccupancyMap::buildModel(){
    for ( int x = -opts_.model_size_; x <= opts_.model_size_; ++x ){
        for ( int y = -opts_.model_size_; y <= opts_.model_size_; ++y ){
            Model2DPoint pt;
            pt.dx_ = x;
            pt.dy_ = y;
            pt.range_ = sqrt(x*x+y*y) * opts_.inv_resolution_;
            pt.angle_ = std::atan2(y,x);
            pt.angle_ = pt.angle_ > M_PI ? pt.angle_ - 2*M_PI : pt.angle_;
            model_.push_back(pt);
        }
    }
}

void OccupancyMap::setPoint( const Vec2i & pt, bool occupy ){
    // int x = pt[0], y = occupancy_grid_.rows - pt[1];
    int x = pt[0], y = pt[1];

    /// 有无 occupied 都要检查，不然会报内存访问错误
    if ( x < 0 ) left_outside_ = true;
    if ( y < 0 ) top_outside_  = true;
    if ( x >= occupancy_grid_.cols )  right_outside_ = true;
    if ( y >= occupancy_grid_.rows ) bottom_outside_ = true;
    if (left_outside_ || top_outside_ || right_outside_ || bottom_outside_ ) return;

    /// 设置上下限
    uchar value = occupancy_grid_.at<uchar>(y,x);
    if ( occupy ) {
        if ( value > 117 ) {
            occupancy_grid_.ptr<uchar>(y)[x] -= 1;
            // occupancy_grid_.at<uchar>(y,x) -= 1;
        }
    } else {
        if (value < 137) {
            occupancy_grid_.ptr<uchar>(y)[x] += 1;
            // occupancy_grid_.at<uchar>(y,x) += 1;
        }
    }
}

double OccupancyMap::findRangeInAngle(double angle, Scan2d::Ptr scan) {
    math::KeepAngleInPI(angle);
    if (angle < scan->angle_min || angle > scan->angle_max) {
        return 0.0;
    }

    int angle_index = int((angle - scan->angle_min) / scan->angle_increment);
    if (angle_index < 0 || angle_index >= scan->ranges.size()) {
        return 0.0;
    }

    int angle_index_p = angle_index + 1;
    double real_angle = angle;

    // take range
    double range = 0;
    if (angle_index_p >= scan->ranges.size()) {
        range = scan->ranges[angle_index];
    } else {
        // 插值
        double s = ((angle - scan->angle_min) / scan->angle_increment) - angle_index;
        double range1 = scan->ranges[angle_index];
        double range2 = scan->ranges[angle_index_p];

        double real_angle1 = scan->angle_min + scan->angle_increment * angle_index;
        double real_angle2 = scan->angle_min + scan->angle_increment * angle_index_p;

        if (range2 < scan->range_min || range2 > scan->range_max) {
            range = range1;
            real_angle = real_angle1;
        } else if (range1 < scan->range_min || range1 > scan->range_max) {
            range = range2;
            real_angle = real_angle2;
        } else if (std::fabs(range1 - range2) > 0.3) {
            range = s > 0.5 ? range2 : range1;
            real_angle = s > 0.5 ? real_angle2 : real_angle1;
        } else {
            range = range1 * (1 - s) + range2 * s;
        }
    }
    return range;
}

void OccupancyMap::bresenhamFilling( const Vec2i & p1, const Vec2i & p2 ){
    int dx = p2.x() - p1.x();
    int dy = p2.y() - p1.y();
    // 方向
    int ux = dx > 0 ? 1 : -1;
    int uy = dy > 0 ? 1 : -1;

    dx = abs(dx);
    dy = abs(dy);
    int x = p1.x();
    int y = p1.y();
    
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

            if (Vec2i(x, y) != p2) {
                setPoint(Vec2i(x, y), false);
            }
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
            if (Vec2i(x, y) != p2) {
                setPoint(Vec2i(x, y), false);
            }
        }
    }
}

void OccupancyMap::dynamicExpand(){
    bool need_expand = left_outside_ || top_outside_ || right_outside_ || bottom_outside_;
    if (!need_expand) {
        return;
    }

    int top_boarder = 0, bottom_boarder = 0, left_boarder = 0, right_boarder = 0;
    if ( left_outside_ ){
        left_boarder = opts_.image_size_ / 5;
        center_image_.x() = center_image_.x() + left_boarder;
    }
    
    if ( top_outside_ ){
        top_boarder = opts_.image_size_ / 5;
        center_image_.y() = center_image_.y() + top_boarder;
    }
    
    if ( right_outside_ ){
        right_boarder = opts_.image_size_ / 5;
    }
    
    if ( bottom_outside_ ){
        bottom_boarder = opts_.image_size_ / 5;
    }

    // cv::Mat使用引用计数，所以赋值操作是浅拷贝，不会复制数据。
    // 因此，可以直接使用赋值操作 occupancy_grid_ = occupancy_grid_copy;，
    // 这样两个矩阵会共享数据，直到其中一个被修改时才会进行深拷贝（写时复制）。
    
    // 执行扩展
    cv::Mat expanded_grid;
    // std::unique_lock<std::mutex> lock(data_mutex_);
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
    left_outside_ = right_outside_ = top_outside_ = bottom_outside_ = false;
}

/// 获取黑白灰形式的占据栅格，作可视化使用
cv::Mat OccupancyMap::getOccupancyGridBlackWhite() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    // 检查网格是否为空
    if (occupancy_grid_.empty()) {
        return cv::Mat(occupancy_grid_.rows, occupancy_grid_.cols, CV_8UC3);
    }

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
