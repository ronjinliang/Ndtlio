#include "../include/ndt_inc.h"
#include "../common/math_utils.h"
#include <glog/logging.h>
#include <set>
#include <execution>
#include <yaml-cpp/yaml.h>

namespace sad {

/// 在voxel里添加点云
void NdtInc2d::addScan( std::shared_ptr<Frame> frame ){
    std::set<KeyType, less_vec<2>> activate_voxels;       // 记录那些 voxel 被更新
    std::vector<Vec2d> out_scan;
    out_scan.resize(frame->pts_.size());

    // 转换到世界坐标
    std::vector<int> index(frame->pts_.size());
    for (int i = 0; i < frame->pts_.size(); ++i) index[i] = i;
    std::for_each(std::execution::par_unseq, index.begin(), index.end(),
        [&]( int & idx ){
            out_scan[idx] = frame->pose_ * frame->pts_[idx];
            // LOG(INFO) << frame->pts_[idx].transpose() << ", " << out_scan[idx].transpose();
        });

    for ( const auto & pt : out_scan ) {
        // 使用体素分辨率计算索引
        KeyType key = (pt * opts_.inv_voxel_size_).array().round().cast<int>();
        auto iter = grids_.find(key);
        if ( iter == grids_.end()) {  // 栅格不存在
            data_.push_front( {key, {pt}} );
            grids_.insert( {key, data_.begin()} );
            
        } else {  // 栅格存在，添加点，更新缓存
            iter->second->second.addPoint(pt);
            // 移动到链表头部（表示最近使用）
            data_.splice(data_.begin(), data_, iter->second);  // 更新的那个放到最前
            // 更新迭代器（因为移动后迭代器可能失效）
            iter->second = data_.begin();                    // grids 时也指向最前
        }
        activate_voxels.emplace(key);
    }

    // 更新 active_voxels
    std::for_each(std::execution::par_unseq, activate_voxels.begin(), activate_voxels.end(),
        [this](const auto & key) { updateVoxel(grids_[key]->second, first_scan_); });
    first_scan_ = false;

    while ( data_.size() >= opts_.capacity_ ) {
        grids_.erase(data_.back().first);  // // 删除最旧的体素（LRU策略）
        data_.pop_back();   //// 删除一个尾部数据
    }
}


bool NdtInc2d::alignNdt( SE2 & init_pose ){
    if (grids_.empty()) {
        LOG(WARNING) << "No grids available for alignment!";
        return false;
    }

    SE2 pose = init_pose;

    // 对点的索引, 预先生成
    std::vector<int> index(source_->pts_.size());
    for (int i = 0; i < source_->pts_.size(); ++i) index[i] = i;

    // 并发代码
    int total_nearby = nearby_grids_.size();
    int total_size = index.size() * total_nearby;

    std::vector<bool> effect_pts(total_size, false);
    // 2x3
    // dex / dtheta  |  dex / dx  |  dex / dy
    // dey / dtheta  |  dey / dx  |  dey / dy
    std::vector<Eigen::Matrix<double, 2, 3>> jacobians(total_size);
    std::vector<Vec2d> errors(total_size);
    std::vector<Mat2d> infos(total_size);
    
    for (int iter = 0; iter < opts_.max_iter_; ++iter) {
        
        // gauss-newtown 迭代
        // 最近邻
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int &idx){
            Vec2d q = source_->pts_[idx];
            Vec2d qs = pose * q;  // 转换到当前估计的位姿

            // 构建雅可比矩阵
            // J = [de/dtheta, de/dx, de/dy]
            // de/dtheta = R * [q_y, -q_x]^T
            // de/dx = [1, 0]^T, de/dy = [0, 1]^T
            // build residual
            Eigen::Matrix<double, 2, 3> J;
            J.block<2, 1>(0,0) = pose.so2().matrix() * Vec2d( -q(1), q(0) );
            J.block<2, 2>(0,1) = Mat2d::Identity();

            // 计算体素索引（使用体素分辨率）
            Vec2i key = (qs*opts_.inv_voxel_size_).array().round().cast<int>();

            for (int i = 0; i < total_nearby; ++i) {
                Vec2i real_key = key + nearby_grids_[i];
                auto it = grids_.find(real_key);
                int real_idx = idx * total_nearby + i;
                /// 检查高斯分布是否已经估计
                if (it != grids_.end() && it->second->second.ndt_estimated_) {
                    auto& v = it->second->second;  // voxel
                    Vec2d e = qs - v.mu_;
                    // check chi2 th 内点, 这里内点非常少
                    // 马氏距离检查（异常值剔除）
                    double mahalanobis  = e.transpose() * v.info_ * e;
                    if ( std::isnan(mahalanobis ) || mahalanobis  > opts_.res_outlier_th_) {
                        effect_pts[real_idx] = false;
                        continue;
                    }

                    jacobians[real_idx] = J;
                    errors[real_idx] = e;
                    infos[real_idx] = v.info_;
                    effect_pts[real_idx] = true;
                } else {
                    effect_pts[real_idx] = false;
                }
            }
        });
        // 累加Hessian和error,计算dx
        // double total_res = 0;
        int effective_num = 0;
        Mat3d H = Mat3d::Zero();
        Vec3d err = Vec3d::Zero();

        for (int idx = 0; idx < effect_pts.size(); ++idx) {
            if (!effect_pts[idx]) {
                continue;
            }

            // total_res += errors[idx].transpose() * infos[idx] * errors[idx];
            effective_num++;

            H += jacobians[idx].transpose() * infos[idx] * jacobians[idx];
            err += -jacobians[idx].transpose() * infos[idx] * errors[idx];
        }

        // 检查有效点数
        if (effective_num < opts_.min_effective_pts_) {
            // LOG(WARNING) << "effective num too small: " << effective_num;
            init_pose = pose;
            return false;
        }

        // 求解增量：H * dx = -err
        Vec3d dx = H.inverse() * err;
        // Vec3d dx = H.ldlt().solve(err);
        // Vec3d dx = H.colPivHouseholderQr().solve(err);
        pose.so2() = pose.so2() * SO2::exp(dx(0));
        pose.translation() += dx.tail<2>();

        if (dx.norm() < opts_.eps_) {
            break;
        }
    }

    init_pose = pose;
    return true;
}

void NdtInc2d::computeResidualAndJacobians( const SE2 & input_pose, Mat8d & HT_Vinv_H, Vec8d & HT_Vinv_r ){
    if (grids_.empty() || source_ == nullptr) {
        LOG(WARNING) << "No grids available for alignment! or source_ is nullptr!";
        return;
    }

    SE2 pose = input_pose;

    // 对点的索引, 预先生成
    std::vector<int> index(source_->pts_.size());
    for (int i = 0; i < source_->pts_.size(); ++i) index[i] = i;

    // 并发代码
    int total_nearby = nearby_grids_.size();
    int total_size = index.size() * total_nearby;

    std::vector<bool> effect_pts(total_size, false);
    std::vector<Eigen::Matrix<double, 2, 8>, Eigen::aligned_allocator<Eigen::Matrix<double, 2, 8>>> jacobians(total_size);
    std::vector<Vec2d> errors(total_size);
    std::vector<Mat2d> infos(total_size);
    
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int & idx){
        Vec2d q = source_->pts_[idx];
        Vec2d qs = pose * q;  // 转换到当前估计的位姿

        // 计算体素索引（使用体素分辨率）
        Vec2i key = (qs*opts_.inv_voxel_size_).array().round().cast<int>();

        for (int i = 0; i < total_nearby; ++i) {
            Vec2i real_key = key + nearby_grids_[i];
            auto it = grids_.find(real_key);
            int real_idx = idx * total_nearby + i;
            /// 检查高斯分布是否已经估计
            if (it != grids_.end() && it->second->second.ndt_estimated_) {
                auto& v = it->second->second;  // voxel
                Vec2d r = qs - v.mu_;    // 对应书上第八章的 residual, error 被定义为 r^2
                // check chi2 th
                // 马氏距离检查（异常值剔除）
                double mahalanobis  = r.transpose() * v.info_ * r;
                if ( std::isnan(mahalanobis ) || mahalanobis  > opts_.res_outlier_th_) {
                    effect_pts[real_idx] = false;
                    continue;
                }

                // 构建雅可比矩阵 这个雅可比矩阵跟优化算法的不同，这个是KF滤波器观测部分的
                //   px  py  vx  vy  R   bg  bax bay
                //   0   1   2   3   4   5   6   7  
                //   ---------------------------------------
                //   1   0   0   0  de1  0   0   0  | 0
                //   0   1   0   0  de2  0   0   0  | 1
                // de/dx = [1, 0]^T, de/dy = [0, 1]^T
                // de/dtheta = R * [q_y, -q_x]^T
                // build residual
                Eigen::Matrix<double, 2, 8> J;
                J.setZero();   // 这里要设为0，否则会出现数值问题, 1e310 这么大的数值
                J.block<2, 2>(0,0) = Mat2d::Identity();
                J.block<2, 1>(0,4) = pose.so2().matrix() * Vec2d( -q(1), q(0) );

                jacobians[real_idx] = J;
                errors[real_idx] = r;
                infos[real_idx] = v.info_;
                
                effect_pts[real_idx] = true;
            } else {
                effect_pts[real_idx] = false;
            }
        }
    });
    // double total_res = 0;
    int effective_num = 0;
    HT_Vinv_H.setZero();
    HT_Vinv_r.setZero();

    for (int idx = 0; idx < effect_pts.size(); ++idx) {
        if (!effect_pts[idx]) continue;

        // total_res += errors[idx].transpose() * infos[idx] * errors[idx];
        effective_num++;
        HT_Vinv_H +=   jacobians[idx].transpose() * infos[idx] * jacobians[idx];
        HT_Vinv_r += - jacobians[idx].transpose() * infos[idx] * errors[idx];
    }
    // LOG(INFO) << "effective: " << effective_num;
}

void NdtInc2d::updateVoxel(Voxel & v, bool & first_scan_flag){
    if ( first_scan_flag ) {
        // 第一帧：简单估计
        if (v.pts_.size() > 1) {
            math::ComputeMeanAndCov(v.pts_, v.mu_, v.sig_, [this](const Vec2d & p) { return p; } );
            // 添加正则化避免奇异
            v.info_ = (v.sig_ + Mat2d::Identity() * opts_.normalizing_factor_ ).inverse();  // 避免出现 nan
        } else {
            v.mu_ = v.pts_[0];
            v.info_ = Mat2d::Identity() * opts_.init_info_;
        }
        v.ndt_estimated_ = true;
        v.pts_.clear();
        return;
    }

    // 非第一帧的处理
    if (v.ndt_estimated_ && v.num_pts_ > opts_.max_pts_in_voxel_) {
        // 点数太多，不再更新（保持稳定）
        v.pts_.clear();  // 释放不再需要的点云内存
        return;
    }

    if (!v.ndt_estimated_ && v.pts_.size() > opts_.min_pts_in_voxel_) {
        // 新体素的初始估计
        math::ComputeMeanAndCov(v.pts_, v.mu_, v.sig_, [this](const Vec2d & p) { return p; });
        v.info_ = (v.sig_ + Mat2d::Identity() * opts_.normalizing_factor_ ).inverse();  // 正则化避免出现 nan
        v.ndt_estimated_ = true;
        v.pts_.clear();
    } else if (v.ndt_estimated_ && v.pts_.size() > opts_.min_pts_in_voxel_) {
        // 增量更新：合并新旧统计数据
        Vec2d cur_mu, new_mu;
        Mat2d cur_var, new_var;
        math::ComputeMeanAndCov(v.pts_, cur_mu, cur_var, [this](const Vec2d & p){return p;});
        // 使用增量公式更新均值和协方差
        math::UpdateMeanAndCov(v.num_pts_, v.pts_.size(), v.mu_, v.sig_, cur_mu, cur_var, new_mu, new_var);
        v.mu_ = new_mu;
        v.sig_ = new_var;
        v.num_pts_ += v.pts_.size();
        v.pts_.clear();

        // 计算新的信息矩阵（稳健的逆）
        Eigen::JacobiSVD<Mat2d> svd(v.sig_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Vec2d lambda = svd.singularValues();
        double max_lambda = lambda[0];
        if (lambda[1] < max_lambda * 1e-4) {
            lambda[1] = max_lambda * 1e-4;
        }
        v.info_ = svd.matrixV() * lambda.cwiseInverse().asDiagonal() * svd.matrixU().transpose();
    }
}

void NdtInc2d::generateNearbyGrids(){
    if (opts_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (opts_.nearby_type_ == NearbyType::NEARBY4) {
        nearby_grids_ = {
            KeyType(0, 0),   // 中心
            KeyType(-1, 0),  // 左
            KeyType(1, 0),   // 右
            KeyType(0, -1),  // 下
            KeyType(0, 1)    // 上
        };
    } else if (opts_.nearby_type_ == NearbyType::NEARBY8) {
        nearby_grids_ = {
            KeyType(0,0),   // 中心
            KeyType(-1,0),  // 左
            KeyType(1,0),   // 右
            KeyType(0,-1),  // 下
            KeyType(0,1),   // 上
            KeyType(-1,-1), // 左下
            KeyType(-1,1),  // 左上
            KeyType(1,-1),  // 右下
            KeyType(1,1)    // 右上
        };
    }
}

}
