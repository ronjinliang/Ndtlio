#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "2dNdtLIO(preintegration)/frame.h"
#include "2dNdtLIO(preintegration)/map.h"

#include "common/eigen_types.h"
#include "common/lidar_utils.h"
#include "opencv2/core/core.hpp"

#include <thread>
#include <atomic>

namespace sad {

class Display {
public:
    Display( const int max_size );
    
    ~Display() = default;

    void setMap( std::shared_ptr<Map> map ) { map_ = map; }

    void close();

    void updateCurrentFrame( std::shared_ptr<Frame> current_frame, bool isKeyframe );

    /// 获取全局地图，不要返回引用
    const cv::Mat getGlobalMap( int size = 500 );

private:
    void displayLoop();
    
    void visualize2DScan(std::vector<Vec2d> & pts, const SE2& pose, cv::Mat& image, const Vec2d & center, float resolution );

private:
    int max_size_ = 500;

    std::thread display_thread_;
    std::mutex data_mutex_;
    std::atomic<bool> display_running_;

    bool isKeyframe_ = false;
    std::shared_ptr<Frame>  current_frame_ = nullptr;
    cv::Mat global_map_;
    
    std::shared_ptr<Map> map_ = nullptr;
};

}


#endif

