//
// Created by xiang on 2021/7/19.
//

#ifndef MAPPING_ODOM_H
#define MAPPING_ODOM_H

namespace sad {

struct Odom {
    Odom() {}
    Odom( double v ) : v_(v) {}  // 我的数据集已经处理好了, 直接发的速度
    Odom(double timestamp, double left_pulse, double right_pulse)
        : timestamp_(timestamp), left_pulse_(left_pulse), right_pulse_(right_pulse) {}

    double timestamp_ = 0.0;
    double left_pulse_ = 0.0;  // 左右轮的单位时间转过的脉冲数
    double right_pulse_ = 0.0;

    double v_ = 0.0;  // 机体系 x 轴方向
};

}  // namespace sad

#endif  // MAPPING_ODOM_H
