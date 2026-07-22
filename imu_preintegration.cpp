#include "2dNdtLIO(preintegration)/imu_preintegration.h"
#include "2dNdtLIO(preintegration)/g2o_types.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

namespace sad{

IMUPreintegration::IMUPreintegration(Options opts, const double & init_bg, const Vec2d & init_ba ) : opts_(opts) {
    const float ng2 = opts_.noise_gyro_ * opts_.noise_gyro_;
    const float na2 = opts_.noise_acce_ * opts_.noise_acce_;
    noise_gyro_acce_.diagonal() << ng2, na2, na2;  //  gz  bax   bay
    
    const double o2 = opts_.odom_var_ * opts_.odom_var_;
    opts_.odom_info_.diagonal() << 1.0 / o2, 1.0 / o2;

    double ga2 = opts_.ndt_ang_noise_ * opts_.ndt_ang_noise_;
    double gp2 = opts_.ndt_pos_noise_ * opts_.ndt_pos_noise_;
    opts_.ndt_info_.diagonal() << 1.0 / ga2, 1.0 / gp2, 1.0 / gp2;

    bg_ = init_bg;
    ba_ = init_ba;

    current_states_.setZero();
    current_states_(5,0) = bg_;
    current_states_.block<2,1>(6,0) = ba_;

    last_states_ = current_states_;
    LOG(INFO) << "noise_gyro_acce: " << "\r\n" << noise_gyro_acce_;
    LOG(INFO) << "ndt_info: " << "\r\n" << opts_.ndt_info_;
}

void IMUPreintegration::setInitCondition( const Options & opts, const double & init_bg, const Vec2d & init_ba ){
    opts_ = opts;

    const float ng2 = opts_.noise_gyro_ * opts_.noise_gyro_;
    const float na2 = opts_.noise_acce_ * opts_.noise_acce_;
    noise_gyro_acce_.diagonal() << ng2, na2, na2;  //  gz  bax   bay
    
    const double o2 = opts_.odom_var_ * opts_.odom_var_;
    opts_.odom_info_.diagonal() << 1.0 / o2, 1.0 / o2;

    double ga2 = opts_.ndt_ang_noise_ * opts_.ndt_ang_noise_;
    double gp2 = opts_.ndt_pos_noise_ * opts_.ndt_pos_noise_;
    opts_.ndt_info_.diagonal() << 1.0 / ga2, 1.0 / gp2, 1.0 / gp2;

    bg_ = init_bg;
    ba_ = init_ba;

    current_states_.setZero();
    current_states_(5,0) = bg_;
    current_states_.block<2,1>(6,0) = ba_;

    last_states_ = current_states_;

    LOG(INFO) << "noise_gyro_acce: " << "\r\n" << noise_gyro_acce_;
    LOG(INFO) << "odom info: " << "\r\n" << opts_.odom_info_;
    LOG(INFO) << "ndt info: " << "\r\n" << opts_.ndt_info_;
}

bool IMUPreintegration::setOdom( const std::shared_ptr<Odom> odom ){
    if ( last_odom_ == nullptr ) {
        last_odom_ = odom;
        return true;
    }
    current_odom_ = odom;
    return true;
}

bool IMUPreintegration::integrate( const IMUPtr imu ){
    double dt = imu->timestamp_ - last_timestamp_;
    if ( dt < 0 || dt > 5*opts_.imu_dt_ ) {
        LOG(INFO) << "skip this imu data because dt " << dt;
        last_timestamp_ = imu->timestamp_;
        return false;
    }
    double dt2 = dt * dt;
    // 去除零偏
    Vec2d imu_acce(imu->acce_.x(), imu->acce_.y());
    Vec2d a = imu_acce - ba_;
    if ( imu->acce_.z() < 2.0 ) {  // 针对归一化之后的数据
        a *= 9.82;
    }
    double gyro = imu->gyro_.z() - bg_;

    Mat2d dR = SO2(dtheta_).matrix();  // 要用到 dR, 先计算 dR 后面再更新 dtheta_
    double omega = gyro * dt;       // 当前角速度 j-1 -> j
    // 更新dv, dp (4.13) (4.16)
    dp_ += dv_ * dt + 0.5 * dR * a * dt2;
    dv_ += dR * a * dt;
    dtheta_ += omega;   // (4.9)
    // dR先不更新，因为A, B, 对bias的雅可比矩阵还需要现在的 dR

    // A 5x5
    // | 1x1    0     0  |
    // | 2x1   I22    0  |
    // | 2x1   2x2   I22 |
    // B 5x3
    // | 1x1   0  |
    // |  0   2x2 |
    // |  0   2x2 |
    Mat5d A;   // 5x5
    A.setIdentity();
    Eigen::Matrix<double, 5, 3> B;  // 5x3
    B.setZero();

    Vec2d acc_hat = Vec2d( -a.y(), a.x() );

    A(0, 0) = 1.0;
    A.block<2, 1>(1, 0) = - dR * dt * acc_hat;
    A.block<2, 1>(3, 0) = -0.5f * dR * acc_hat * dt2;
    A.block<2, 2>(3, 1) =  dt * Mat2d::Identity();
    
    // B(0, 0) = dt;             // 2d 情况 Jr = 1
    B(0, 0) = -dt;             // 2d 情况 Jr = 1   TUDO
    B.block<2, 2>(1, 1) = dR * dt;
    B.block<2, 2>(3, 1) = 0.5 * dR * dt2;

    // 对bias的雅可比矩阵
    dp_dba_ = dp_dba_ + dv_dba_ * dt - 0.5f * dt2 * dR;                      // (4.39d)
    dp_dbg_ = dp_dbg_ + dv_dbg_ * dt - 0.5f * dt2 * dR * acc_hat * dR_dbg_;  // (4.39e)
    dv_dba_ = dv_dba_ - dR * dt;                                             // (4.39b)
    dv_dbg_ = dv_dbg_ - dR * dt * acc_hat * dR_dbg_;                         // (4.39c)
    // 更新 dR_dbg
    // dR_dbg_ = omega * dR_dbg_ - dt;                         // (4.39a)   TUDO  dR_dbg_ = dR_dbg_ - dt;
    dR_dbg_ = dR_dbg_ - dt;

    // 更新协方差矩阵, 为优化提供信息矩阵
    cov_ = A * cov_ * A.transpose() + B * noise_gyro_acce_ * B.transpose();

    // 预积分时间
    dt_ += dt;
    last_timestamp_ = imu->timestamp_;  /// 更新时间
    return true;
}

SO2 IMUPreintegration::getDeltaR( const double &bg ){
    return SO2::exp(dtheta_) * SO2::exp( dR_dbg_ * ( bg - bg_ ) );  // bg - bg_ =  δbg
}

Vec2d IMUPreintegration::getDeltaV(const double &bg, const Vec2d &ba){
    return dv_ + dv_dbg_ * ( bg - bg_ ) + dv_dba_ * ( ba - ba_ );
}

Vec2d IMUPreintegration::getDeltaP(const double &bg, const Vec2d &ba){
    return dp_ + dp_dbg_ * ( bg - bg_ ) + dp_dba_ * ( ba - ba_ );
}

Vec8d IMUPreintegration::predict( const Vec8d &start ) const {
    // 0 1 2 3  4  5   6   7
    // θ x y vx vy bg bax bay
    Vec8d end;
    Mat2d startR = SO2(start(0)).matrix();
    end.block<2,1>(1,0) = startR * dp_ + start.block<2,1>(3,0) * dt_ + start.block<2,1>(1,0);
    end.block<2,1>(3,0) = startR * dv_ + start.block<2,1>(3,0);
    end(0) = start(0) + dtheta_;
    end.block<3,1>(5,0) = start.block<3,1>(5,0);
    return end;
}

void IMUPreintegration::optimize( SE2 & ndt_pose ){
    /// NOTE 这些东西是对参数非常敏感的。相差几个数量级的话，容易出现优化不动的情况
    using BlockSolverType = g2o::BlockSolverX;
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

    // auto *solver = new g2o::OptimizationAlgorithmLevenberg(
    //     g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    
    auto linearSolver = std::make_unique<LinearSolverType>();   // 创建线性求解器
    auto blockSolver = std::make_unique<BlockSolverType>(std::move(linearSolver)); // 创建块求解器
    auto* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver)); // 创建优化算法

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // 0 1 2 3  4  5   6   7
    // θ x y vx vy bg bax bay

    // 上时刻顶点， pose, v, bg, ba
    auto v0_pose = new VertexSE2();
    v0_pose->setId(0);
    v0_pose->setEstimate( SE2( last_states_(0), last_states_.block<2,1>(1,0) ) );
    optimizer.addVertex(v0_pose);

    auto v0_vel = new VertexVelocity();
    v0_vel->setId(1);
    v0_vel->setEstimate( last_states_.block<2,1>(3,0) );
    optimizer.addVertex(v0_vel);

    auto v0_bg = new VertexGyroBias();
    v0_bg->setId(2);
    v0_bg->setEstimate( last_states_(5) );
    optimizer.addVertex(v0_bg);

    auto v0_ba = new VertexAcceBias();
    v0_ba->setId(3);
    v0_ba->setEstimate( last_states_.block<2,1>(6,0) );
    optimizer.addVertex(v0_ba);

    // 本时刻顶点，pose, v, bg, ba
    auto v1_pose = new VertexSE2();
    v1_pose->setId(4);
    v1_pose->setEstimate( ndt_pose );  // NDT pose作为初值
    // v1_pose->setEstimate(current_nav_state_.GetSE3());  // 预测的pose作为初值
    optimizer.addVertex(v1_pose);

    auto v1_vel = new VertexVelocity();
    v1_vel->setId(5);
    v1_vel->setEstimate( current_states_.block<2,1>(3,0) );
    optimizer.addVertex(v1_vel);

    auto v1_bg = new VertexGyroBias();
    v1_bg->setId(6);
    v1_bg->setEstimate( current_states_(5) );
    optimizer.addVertex(v1_bg);

    auto v1_ba = new VertexAcceBias();
    v1_ba->setId(7);
    v1_ba->setEstimate( current_states_.block<2,1>(6,0) );
    optimizer.addVertex(v1_ba);

    // imu factor
    auto edge_inertial = new EdgeInertial( *this, opts_.info_wright_ );
    edge_inertial->setVertex(0, v0_pose);
    edge_inertial->setVertex(1, v0_vel);
    edge_inertial->setVertex(2, v0_bg);
    edge_inertial->setVertex(3, v0_ba);
    edge_inertial->setVertex(4, v1_pose);
    edge_inertial->setVertex(5, v1_vel);
    auto *rk = new g2o::RobustKernelHuber();
    rk->setDelta(200.0);
    edge_inertial->setRobustKernel(rk);
    optimizer.addEdge(edge_inertial);

    // 零偏随机游走
    auto *edge_gyro_rw = new EdgeGyroRW();
    edge_gyro_rw->setVertex(0, v0_bg);
    edge_gyro_rw->setVertex(1, v1_bg);
    edge_gyro_rw->setInformation(opts_.bg_rw_info_);
    optimizer.addEdge(edge_gyro_rw);

    auto *edge_acc_rw = new EdgeAcceRW();
    edge_acc_rw->setVertex(0, v0_ba);
    edge_acc_rw->setVertex(1, v1_ba);
    edge_acc_rw->setInformation(opts_.ba_rw_info_);
    optimizer.addEdge(edge_acc_rw);

    // 上一帧pose, vel, bg, ba的先验
    auto *edge_prior = new EdgePriorPoseNavState( last_states_, prior_info_);
    edge_prior->setVertex(0, v0_pose);
    edge_prior->setVertex(1, v0_vel);
    edge_prior->setVertex(2, v0_bg);
    edge_prior->setVertex(3, v0_ba);
    optimizer.addEdge(edge_prior);

    /// 使用NDT的pose进行观测
    auto *edge_ndt = new EdgeGNSS(v1_pose, ndt_pose);
    edge_ndt->setInformation(opts_.ndt_info_);
    optimizer.addEdge(edge_ndt);

    if ( opts_.odom_var_ > 0 ) {  // yaml 文件中设置为正数就加入 odom 观测
        // 使用 odom 观测  上一时刻
        last_odom_speed_ = SO2(last_states_(0)) * Vec2d( last_odom_->v_, 0 );
        auto * edge_last_odom = new EdgeEncoder2D( v0_vel, last_odom_speed_ );  // 因为传递引用, 要用变量临时存储起来
        edge_last_odom->setInformation(opts_.odom_info_);
        optimizer.addEdge(edge_last_odom);
        
        // 使用 odom 观测  当前时刻
        current_odom_speed_ = SO2(current_states_(0)) * Vec2d( current_odom_->v_, 0 );
        auto * edge_current_odom = new EdgeEncoder2D( v1_vel, current_odom_speed_ );
        edge_current_odom->setInformation(opts_.odom_info_);
        optimizer.addEdge(edge_current_odom);
    }


    if ( !opts_.update_bias_gyro_ ) {
        v0_bg->setFixed(true);
    }
    if ( !opts_.update_bias_acce_ ) {
        v0_ba->setFixed(true);
    }

    // go
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    ndt_pose = v1_pose->estimate();

    // get results
    last_states_(0) = v0_pose->estimate().so2().log();
    last_states_.block<2,1>(1,0) = v0_pose->estimate().translation();
    last_states_.block<2,1>(3,0) = v0_vel->estimate();
    last_states_(5) = v0_bg->estimate();
    last_states_.block<2,1>(6,0) = v0_ba->estimate();

    current_states_(0) = v1_pose->estimate().so2().log();
    current_states_.block<2,1>(1,0) = v1_pose->estimate().translation();
    current_states_.block<2,1>(3,0) = v1_vel->estimate();
    current_states_(5) = v1_bg->estimate();
    current_states_.block<2,1>(6,0) = v1_ba->estimate();

    // if (opts_.verbose_) {
    //     LOG(INFO) << "last changed to: " << last_states_.transpose();
    //     LOG(INFO) << "curr changed to: " << current_states_.transpose();
    //     LOG(INFO) << "preinteg chi2: " << edge_inertial->chi2() << ", err: " << edge_inertial->error().transpose();
    //     LOG(INFO) << "prior chi2: " << edge_prior->chi2() << ", err: " << edge_prior->error().transpose();
    //     LOG(INFO) << "ndt: " << edge_ndt->chi2() << "/" << edge_ndt->error().transpose();
    // }

    /// 重置预积分
    reset();

    // 计算当前时刻先验
    // 构建hessian
    // 8x2，顺序: v0_pose, v0_vel, v0_bg, v0_ba, v1_pose, v1_vel, v1_bg, v1_ba
    //              0        3      5      6       8        11     13     14
    Eigen::Matrix<double, 16, 16> H;
    H.setZero();

    H.block<13, 13>(0, 0) += edge_inertial->GetHessian();  // 13x13

    Mat2d Hgr = edge_gyro_rw->GetHessian();   // 2x2
    H.block<1, 1>(5, 5)   += Hgr.block<1, 1>(0, 0);  // v0_bg | v0_bg
    H.block<1, 1>(5, 13)  += Hgr.block<1, 1>(0, 1);  // v0_bg | v1_bg
    H.block<1, 1>(13, 5)  += Hgr.block<1, 1>(1, 0);  // v1_bg | v0_bg
    H.block<1, 1>(13, 13) += Hgr.block<1, 1>(1, 1);  // v1_bg | v1_bg

    Mat4d Har = edge_acc_rw->GetHessian();  // 4x4
    H.block<2, 2>(6, 6)   += Har.block<2, 2>(0, 0);  // v0_ba | v0_ba
    H.block<2, 2>(6, 14)  += Har.block<2, 2>(0, 2);  // v0_ba | v1_ba
    H.block<2, 2>(14, 6)  += Har.block<2, 2>(2, 0);  // v1_ba | v0_ba
    H.block<2, 2>(14, 14) += Har.block<2, 2>(2, 2);  // v1_ba | v1_ba

    H.block<8, 8>(0, 0) += edge_prior->GetHessian();  // 8x8  v0_pose, v0_vel, v0_bg, v0_ba
    H.block<3, 3>(8, 8) += edge_ndt->GetHessian();    // 3x3  v1_pose | v1_pose

    H = math::Marginalize(H, 0, 7);      // 边缘化 v0_pose, v0_vel, v0_bg, v0_ba
    prior_info_ = H.block<8, 8>(8, 8);

    // NormalizeVelocity();
    last_odom_ = current_odom_;
    last_states_ = current_states_;
}

void IMUPreintegration::reset(){
    dt_ = 0.;    // 预积分时间
    // 预积分观测量
    dtheta_ = 0.0;
    dv_ = Vec2d::Zero();
    dp_ = Vec2d::Zero();
    // 零偏
    bg_ = current_states_(5,0);
    ba_ = current_states_.block<2,1>(6,0);

    cov_.setZero();    // 噪声协方差矩阵 ∑ = A ∑ A^T + B Cov(n_d) B^T (4.31)

    dR_dbg_ = 0.0;  // 1x1
    dv_dba_.setZero();
    dv_dbg_.setZero();
    dp_dba_.setZero();
    dp_dbg_.setZero();
}

}

