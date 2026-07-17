#ifndef __OCCUPANCY_MAP_H
#define __OCCUPANCY_MAP_H

#include "../include/frame.h"
#include "../common/eigen_types.h"

#include <opencv2/core.hpp>

#include <thread>
#include <map>

namespace sad {

/**
 * 栅格占据地图类
 * 
 */
class OccupancyMap {
public:
    struct Options {
        // 参数
        double closest_th_ = 0.2;         // 近距离阈值
        double endpoint_close_th_ = 0.1;  // 末端点障碍物近距离阈值
        double resolution_ = 20.0;        // 1m 多少像素
        float inv_resolution_ = 1./resolution_;     // 1个像素多少米（栅格分辨率）
        int image_size_ = 1000;           // 图像大小
    };

    struct OutsideFlags {
        bool left_outside_   = false;
        bool right_outside_  = false;
        bool top_outside_    = false;
        bool bottom_outside_ = false;
    };

    OccupancyMap(){
        occupancy_grid_ = cv::Mat(opts_.image_size_, opts_.image_size_, CV_8U, 127);  // 8bit占据栅格图像
        center_image_ = Vec2d( opts_.image_size_ / 2, opts_.image_size_ / 2 );
    }

    void loadParams( const Options & opts ){
        opts_ = opts;
        occupancy_grid_ = cv::Mat(opts_.image_size_, opts_.image_size_, CV_8U, 127);  // 8bit占据栅格图像
        center_image_ = Vec2d( opts_.image_size_ / 2, opts_.image_size_ / 2 );
    }

    /// 往这个占据栅格地图中增加一个frame
    /// @brief 用 bresenhamFilling 直线填充
    /// @param frame 
    void addLidarFrame( std::shared_ptr<Frame> frame );

    /// @brief 回环检测中给缓冲区添加帧
    /// @param frame 
    void addLidarFrameInBuffer( std::map<size_t, std::shared_ptr<Frame>> & frames, const int & frame_gap );

    // 获取原始占据栅格地图
    cv::Mat getOccupancyGrid() const { return occupancy_grid_; }
    /// 获取黑白灰形式的占据栅格，作可视化使用
    cv::Mat getOccupancyGridBlackWhite();
    Vec2d getCenter() { return center_image_; }
    float getResolution() { return opts_.resolution_; }

private:

    /// 从世界坐标系转到图像坐标系
    template <class T>
    inline Vec2i world2Image(const Eigen::Matrix<T, 2, 1>& pt) {
        Vec2d pt_map = pt * opts_.resolution_ + center_image_;
        int x = int(pt_map[0]);  // 扩展图像会出现偏移
        int y = int(pt_map[1]);   // y轴方向换一下
        return Vec2i(x, y);
    }
    

    /**
     * Bresenham直线填充，给定起始点和终止点，将中间的区域填充为白色
     * @param p1
     * @param p2
     */
    void bresenhamFilling( const Vec2i & p1, const Vec2i & p2 );
    void dynamicExpand();
    /// 在某个点填入占据或者非占据信息
    void setPoint( const Vec2i & pt, bool occupy );

private:
    std::mutex data_mutex_;

    Options opts_;

    // 标注栅格化过程中是否有落在外部的点
    OutsideFlags outFlags_;
    cv::Mat occupancy_grid_;
    Vec2d center_image_;
};

}


#endif // __OCCUPANCY_MAP_H

