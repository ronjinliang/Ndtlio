#ifndef __NDT_INC_H
#define __NDT_INC_H

#include "../common/eigen_types.h"

#include "../include/frame.h"

#include <list>
#include <mutex>

namespace sad {

class NdtInc2d {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum class NearbyType {
        CENTER,   // 只使用中心网格
        NEARBY4,  // 结构化环境，4邻域足够
        NEARBY8   // 非结构化环境，需要8邻域
    };

    struct Options {
        int max_iter_ = 50;                      // 最大迭代次数
        double voxel_size_ = 0.25;                // 体素大小   0.1 m
        double inv_voxel_size_ = 1./voxel_size_; // 体素大小的倒数  voxel / m
        int min_effective_pts_ = 10;              // 最近邻点数阈值
        int min_pts_in_voxel_ = 5;               // 每个栅格中最小点数
        int max_pts_in_voxel_ = 50;              // 每个栅格中最大点数
        double eps_ = voxel_size_ * 0.01;        // 通常取体素大小的1%
        double res_outlier_th_ = 5.991;          // 卡方检验阈值（95%置信度）  基于马氏距离的卡方检验阈值。
        double normalizing_factor_ = 1e-3;        // 正则化项，防止矩阵求逆出现 nan
        double init_info_ = 1e2;
        size_t capacity_ = 100000;               // 缓存的体素数量
        NearbyType nearby_type_ = NearbyType::NEARBY8;
    };

    /// 体素内部结构
    struct Voxel {
        Voxel() = default;
        Voxel( const Vec2d & pt ) {
            pts_.emplace_back(pt);
            num_pts_ = 1;
        }

        void addPoint( const Vec2d& pt ) {
            pts_.emplace_back(pt);
            if (!ndt_estimated_) num_pts_++;
        }

        std::vector<Vec2d> pts_;    // 内部点，多于一定数量之后再估计均值和协方差
        Vec2d mu_ = Vec2d::Zero();  // 均值
        Mat2d sig_ = Mat2d::Zero(); // 协方差
        Mat2d info_ = Mat2d::Zero();// 协方差矩阵的逆
        bool ndt_estimated_ = false;// NDT 是否已经估计
        int num_pts_ = 0;           // 总点数
    };
    
    using KeyType = Vec2i;   // 体素索引
    using KeyAndVoxel = std::pair<KeyType, Voxel>;

    NdtInc2d() {
        opts_.inv_voxel_size_ = 1. / opts_.voxel_size_;
        generateNearbyGrids();
    }
    
    NdtInc2d( Options opts ) : opts_(opts) {
        opts_.inv_voxel_size_ = 1. / opts_.voxel_size_;
        generateNearbyGrids();
    }

    void loadParams( const Options & opts ) { opts_ = opts; }
    
    /// 设置被配准的Scan
    void setSource( std::shared_ptr<Frame> source ) { source_ = source; }
    
    /// 获取一些统计信息
    int getNumGrids() const { return grids_.size(); }

    /// 在voxel里添加点云
    void addScan( std::shared_ptr<Frame> frame );

    /// 使用gauss-newton方法进行ndt配准, LO 或者松耦合 LIO 使用
    bool alignNdt( SE2 & init_pose );

    /**
     * 计算给定Pose下的雅可比和残差矩阵，符合IEKF中符号（8.17, 8.19）
     * 实现紧耦合 IESKF LIO
     * @param pose
     * @param HTVH
     * @param HTVr
     */
    void computeResidualAndJacobians( const SE2 & input_pose, Mat8d & HT_Vinv_H, Vec8d & HT_Vinv_r );

private:
    //// 更新 voxel 内部数据，根据新加入的 pts 和历史估计情况来确定自己的估计
    void updateVoxel(Voxel & v, bool & first_scan_flag);
    /// 根据最近邻的类型，生成附近网格
    void generateNearbyGrids();

private:
    Options opts_;
    std::shared_ptr<Frame> source_ = nullptr;
    std::vector<KeyType> nearby_grids_;     // 附近的栅格

    // 哈希表：
    // 1.键（KeyType）：体素索引
    // 2.值（std::list<KeyAndVoxel>::iterator）：指向一个链表（std::list）中元素的迭代器。
    // 3.哈希函数（hash_vec<2>）：因为键是Vec2i（Eigen::Vector2i），需要自定义哈希函数。hash_vec<2>是一个模板类，用于生成二维向量的哈希值
    // grids_只存储索引到迭代器的映射
    std::list<KeyAndVoxel> data_;                      // 缓存数据
    std::unordered_map<KeyType, std::list<KeyAndVoxel>::iterator, hash_vec<2>> grids_;  // 栅格数据，存储真实数据的迭代器
    bool first_scan_ = true;  // 首帧点云特殊处理
};

}




#endif
