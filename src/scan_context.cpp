#include "2dNdtLIO/include/scan_context.h"
#include <cmath>
#include <algorithm>
#include <execution>

namespace sad {

ScanContext2D::ScanContext2D( const Options & opts ) : opts_(opts) {
    ring_step_ = ( opts_.max_range_ - opts_.min_range_ ) / opts_.num_rings_;
    sector_step_ = 2.0 * M_PI / opts_.num_sectors_;
}

//// 从 2d 点云计算 Scan Context 描述子
Eigen::MatrixXd ScanContext2D::compute( const std::vector<Vec2d> & points ){
    Eigen::MatrixXd sc = Eigen::MatrixXd::Zero( opts_.num_rings_, opts_.num_sectors_ );  // x y

    // 下采样

    // 转为极坐标
    std::vector<size_t> index(points.size());
    for ( size_t i = 0; i < points.size(); ++ i ) index[i] = i;

    std::vector<Vec2d> polar_points(points.size());
    std::for_each( std::execution::par_unseq, index.begin(), index.end(), 
        [points, polar_points] ( const size_t & i ) {
        Vec2d pt = points[i];
        double range = pt.norm();
        double angle = std::atan2( pt.y(), pt.x() );  // [-pi, pi]
        polar_points[i](range, angle);
    } );

    // 填充 scan context 矩阵
    for ( const auto & pt : polar_points ) {
        double range = pt[0];
        double angle = pt[1];
        if ( range < opts_.min_range_ || range > opts_.max_range_ ) continue;

        // 计算环索引
        int ring_idx = static_cast<int>( ( range - opts_.min_range_ ) / ring_step_ );
        if ( ring_idx >= opts_.num_rings_ ) ring_idx = opts_.num_rings_ - 1;

        // 计算扇区索引 角度归一化到[0, 2pi]
        if ( angle < 0 ) angle += 2 * M_PI;
        int sector_idx = static_cast<int>( angle / sector_step_ );
        if ( sector_idx >= opts_.num_sectors_ ) sector_idx = opts_.num_sectors_ - 1;

        // 填充值：最大距离 或者平均距离
        if ( range > sc(ring_idx, sector_idx) ) {
            sc(ring_idx, sector_idx) = range;
        }
    }

    return sc;
}

/// 计算两个 Scan Context 之间的距离(环状关键点匹配)
double ScanContext2D::distance( const Eigen::MatrixXd & sc1, const Eigen::MatrixXd & sc2 ){
    if ( sc1.rows() != sc2.rows() || sc1.cols() != sc2.cols() ) return std::numeric_limits<double>::max();

    double min_dist = std::numeric_limits<double>::max();

    // 环状关键点匹配  旋转不变性
    for ( int shift = 0; shift < sc1.cols(); ++ shift ) {
        double dist = 0.0;
        int valid_cells = 0;

        for ( int i = 0; i < sc1.rows(); ++ i ) {
            for ( int j = 0; j < sc1.cols(); ++ j ) {
                int shifted_j = ( j + shift ) % sc1.cols();

                double val1 = sc1(i, j);
                double val2 = sc2(i, shifted_j);

                // 忽略空单元格
                if ( val1 < 1e-6 || val2 < 1e-6 ) continue;

                dist += std::abs(val1 - val2);
                ++valid_cells;
            }
        }

        if ( valid_cells > 0 ) {
            dist /= valid_cells;
            if ( dist < min_dist ) min_dist = dist;
        }
    }
    return min_dist;
}

/// 快速匹配(主成分向量)
double ScanContext2D::fastDistance( const Eigen::MatrixXd & sc1, const Eigen::MatrixXd & sc2 ){
    Eigen::VectorXd ring_key1 = computeRingKey( sc1 );
    Eigen::VectorXd ring_key2 = computeRingKey( sc2 );

    return ( ring_key1 - ring_key2 ).norm();
}

/// 计算描述子的主成分向量，用于快速搜索
Eigen::VectorXd ScanContext2D::computeRingKey( const Eigen::MatrixXd & sc ){
    /// summary: rowwise mean vector 行均值
    // col x -> i, row y -> j
    Eigen::VectorXd ring_key = Eigen::VectorXd::Zero( opts_.num_rings_ );

    for ( int i = 0; i < opts_.num_rings_; ++i ) {
        double sum = 0.0;
        int valid_sectors = 0;

        for ( int j = 0; j < opts_.num_sectors_; ++j ) {
            if ( sc(i, j) > 1e-6 ) {
                sum += sc(i, j);
                ++valid_sectors;
            }
        }
        if ( valid_sectors > 0 ) {
            ring_key(i) = sum / valid_sectors;
        }
    }

    return ring_key;
}

/// 可视化 Scan context
// cv::Mat visualize(const Eigen::MatrixXd& sc);

}


