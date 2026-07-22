#ifndef __SCAN_CONTEXT_H
#define __SCAN_CONTEXT_H

#include "2dNdtLIO/common/eigen_types.h"
#include "2dNdtLIO/include/frame.h"
#include <vector>
#include <memory>
#include <thread>
#include <map>
#include <nanoflann.hpp>

namespace sad {

class ScanContext2D {
public:
    struct Options {
        int num_rings_ = 20;          // 径向分段数量
        int num_sectors_ = 60;        // 角度分段数量
        double max_range_ = 80.0;     // 最大检测距离（米）
        double min_range_ = 1.0;      // 最小检测距离（米）
        int down_sample_factor_ = 1;  // 下采样因子
    };

    ScanContext2D( const Options & opts );

    //// 从 2d 点云计算 Scan Context 描述子
    Eigen::MatrixXd compute( const std::vector<Vec2d> & points );

    /// 计算两个 Scan Context 之间的距离(环状关键点匹配)
    double distance( const Eigen::MatrixXd & sc1, const Eigen::MatrixXd & sc2 );
    
    /// 快速匹配(主成分向量)
    double fastDistance( const Eigen::MatrixXd & sc1, const Eigen::MatrixXd & sc2 );

    /// 计算描述子的主成分向量，用于快速搜索
    Eigen::VectorXd computeRingKey( const Eigen::MatrixXd & sc );

    // Eigen::VectorXd computeSectorKey( const Eigen::MatrixXd & sc );

    /// 可视化 Scan context
    // cv::Mat visualize(const Eigen::MatrixXd& sc);

private:
    Options opts_;
    double ring_step_;    // 每个环的距离步长
    double sector_step_;  // 每个扇区的角度步长
};

class ScanContextManager {
public:
    //// 数据库条目
    struct Data {
        size_t kf_id_;
        Eigen::MatrixXd descriptor_;        // 完整的 Scan Context 描述子
        Eigen::VectorXd ring_key_;          // 环状关键向量（用于 KD-Tree）

        Data(size_t & kf_id, Eigen::MatrixXd & descriptor, Eigen::VectorXd & ring_key)
        : kf_id_(kf_id), descriptor_(descriptor), ring_key_(ring_key) {}
    };

    explicit ScanContextManager( ScanContext2D::Options & sc_opts );
    
    /**
     * @brief 添加一个关键帧到数据库
     * @param frame_id 帧ID
     * @param points   该帧的点云（用于计算 Scan Context）
     * @param pose     帧的位姿（可选）
     */
    void addFrame( std::shared_ptr<Frame> kf );

    /**
     * @brief 检索与查询点云最相似的 Top-K 候选帧
     * @param query_points  查询点云
     * @param query_frame_id  查询帧的ID（用于排除自身）
     * @param k_nearest     返回的最邻近数量（KD-Tree 初筛数）
     * @param sc_threshold  完整 Scan Context 距离阈值（过滤不相似的候选）
     * @return std::vector<size_t>  候选帧 ID 列表（已按相似度排序）
     */
    std::vector<size_t> search(const std::vector<Vec2d>& query_points,
                                size_t query_frame_id,
                                int k_nearest = 5,
                                double sc_threshold = 2.0);
    
    /**
     * @brief 根据帧ID获取存储的数据
     */
    const Data* getData(size_t frame_id) const;

    /**
     * @brief 获取内部 Scan Context 检测器（用于计算）
     */
    const ScanContext2D& getDetector() const { return *detector_; }

    // ---------- 调试接口 ----------
    size_t size() const { std::lock_guard<std::mutex> lock(mutex_); return data_map_.size(); }

private:
    // 为了 nanoflann 适配，需要提供访问 Ring Key 的接口
    // 我们将所有 ring_key 打包成一个 Eigen::MatrixXd (N x dim)
    // 并实现一个适配器类
    struct RingKeyAdaptor {
        const Eigen::MatrixXd& mat;   // 每行是一个 ring_key
        RingKeyAdaptor(const Eigen::MatrixXd& m) : mat(m) {}
        inline size_t kdtree_get_point_count() const { return mat.rows(); }
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
            return mat(idx, dim);
        }
        template <class BBOX>
        bool kdtree_get_bbox(BBOX&) const { return false; }
    };

    // 更新 KD-Tree（当数据变化时调用，目前只有添加操作，可增量重建）
    void rebuildKDTree();

private:
    mutable std::mutex mutex_;                           // 保护数据库
    std::unique_ptr<ScanContext2D> detector_;            // Scan Context 计算器
    std::map<size_t, Data> data_map_;                    // ID -> Data
    Eigen::MatrixXd ring_key_matrix_;                     // 所有 ring_key 组成的矩阵 (N x dim)
    std::vector<size_t> ring_key_to_id_;                  // 矩阵行索引 -> 帧ID
    using KDTree = nanoflann::KDTreeEigenMatrixAdaptor<RingKeyAdaptor, 3, nanoflann::metric_L2>;
    std::unique_ptr<KDTree> kdtree_;                       // KD-Tree 索引
    bool need_rebuild_ = false;                            // 标记是否需要重建 KD-Tree

};

}


#endif
