#ifndef LIDAR_2D_UTILS_H
#define LIDAR_2D_UTILS_H

#include "common/eigen_types.h"
#include "common/lidar_utils.h"
#include "opencv2/core/core.hpp"

namespace sad {

/**
 * 在image上绘制一个2D scan
 * @param scan
 * @param pose
 * @param image
 * @param image_size 图片大小
 * @param resolution 分辨率，一米多少个像素
 * @param pose_submap 如果是子地图，提供子地图的pose
 */
void Visualize2DScan(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, 
                    const Vec3b& color, int image_size = 800,
                     float resolution = 20.0, const SE2& pose_submap = SE2());
}


#endif

