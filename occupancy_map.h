#ifndef __SUBMAP_H
#define __SUBMAP_H

#include "2dNdtLIO(preintegration)/frame.h"

#include "common/eigen_types.h"
#include "common/lidar_utils.h"

#include <opencv2/core.hpp>

#include <thread>

namespace sad {

/**
 * 栅格占据地图类
 * 
 */
class OccupancyMap {
public:
    enum class GridMethod {
        MODEL_POINTS,  // 模板化算法
        BRESENHAM,     // 直接栅格化算法
    };

    struct Options {
        // 参数
        double closest_th_ = 0.2;         // 近距离阈值
        double endpoint_close_th_ = 0.1;  // 末端点障碍物近距离阈值
        double resolution_ = 20.0;        // 1m 多少像素
        float inv_resolution_ = 1./resolution_;     // 1个像素多少米（栅格分辨率）
        int image_size_ = 1000;           // 图像大小
        int model_size_ = 400;            // 模板像素大小
        GridMethod method_ = GridMethod::BRESENHAM;
    };

    struct Model2DPoint{
        int dx_{0}, dy_{0};
        double angle_{0.0};
        float range_{0.0};
    };


    OccupancyMap(){
        buildModel();
        occupancy_grid_ = cv::Mat(opts_.image_size_, opts_.image_size_, CV_8U, 127);  // 8bit占据栅格图像
        center_image_ = Vec2d( opts_.image_size_ / 2, opts_.image_size_ / 2 );
    }

    void loadParams( const Options & opts ){
        opts_ = opts;
        buildModel();  // 加载参数之后重新建立模板
        occupancy_grid_ = cv::Mat(opts_.image_size_, opts_.image_size_, CV_8U, 127);  // 8bit占据栅格图像
        center_image_ = Vec2d( opts_.image_size_ / 2, opts_.image_size_ / 2 );
    }

    /// 往这个占据栅格地图中增加一个frame
    void addLidarFrame( std::shared_ptr<Frame> frame, Scan2d::Ptr scan );

    /// 在某个点填入占据或者非占据信息
    void setPoint( const Vec2i & pt, bool occupy );

    /// 设置中心点
    void setPose(const SE2& pose) { pose_ = pose; }

    // 获取原始占据栅格地图
    cv::Mat getOccupancyGrid() const { return occupancy_grid_; }
    
    /// 获取黑白灰形式的占据栅格，作可视化使用
    cv::Mat getOccupancyGridBlackWhite();

    Vec2d getCenter() { return center_image_; }
    float getResolution() { return opts_.resolution_; }

private:
    /// 生成填充w模板
    void buildModel();

    /// 从世界坐标系转到图像坐标系
    template <class T>
    inline Vec2i world2Image(const Eigen::Matrix<T, 2, 1>& pt) {
        Vec2d pt_map = (pose_.inverse() * pt) * opts_.resolution_ + center_image_;
        int x = int(pt_map[0]);  // 扩展图像会出现偏移
        int y = int(pt_map[1]);   // y轴方向换一下
        return Vec2i(x, y);
    }

    /// 查找某个角度下的range值
    double findRangeInAngle(double angle, Scan2d::Ptr scan);

    /**
     * Bresenham直线填充，给定起始点和终止点，将中间的区域填充为白色
     * @param p1
     * @param p2
     */
    void bresenhamFilling( const Vec2i & p1, const Vec2i & p2 );

    void dynamicExpand();

private:
    std::mutex data_mutex_;

    Options opts_;
    SE2 pose_;  // T_W_S

    // 标注栅格化过程中是否有落在外部的点
    bool left_outside_   = false;
    bool right_outside_  = false;
    bool top_outside_    = false;
    bool bottom_outside_ = false;
    cv::Mat occupancy_grid_;

    // 模板
    std::vector<Model2DPoint> model_; // 填充占据栅格的模板，世界系的点

    Vec2d center_image_;
};

}


#endif // __SUBMAP_H

