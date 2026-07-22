#include <yaml-cpp/yaml.h>
#include "2dNdtLIO/include/loop_closure.h"
#include "2dNdtLIO/common/g2o_types.h"

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

namespace sad {

LoopClosure::LoopClosure( Options opts ) : opts_(opts) {
    
    debug_fout.open( opts_.debug_fout_ );

    thread_running_.store(true);
    thread_ = std::thread(std::bind(&LoopClosure::loop, this));
    LOG(INFO) << "Created loop closing thread.";
}

void LoopClosure::addNewFrame( std::shared_ptr<Frame> frame ){
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        current_add_frame_ = frame;
        has_new_frame_ = true;
    }
    thread_step_.notify_one();
}

void LoopClosure::close(){
    thread_running_.store(false);
    thread_step_.notify_all();
    LOG(INFO) << "Closing loop closing thread...";
    if (thread_.joinable()) {
        thread_.join();
    }
    LOG(INFO) << "Closed loop closing.";
    if (debug_fout.is_open()) {
        debug_fout.close();
    }
}

bool LoopClosure::detect(){
    has_new_frame_ = false;
    has_new_loops_ = false;
    std::unique_lock<std::mutex> lock(data_mutex_);
    thread_step_.wait(lock, [this](){ return !thread_running_.load() || has_new_frame_; });

    current_frame_ = current_add_frame_;
    frames_ = map_->getAllFramesByCopy();   // 深度 copy 所有帧
    lock.unlock();

    if ( current_frame_->keyframe_id_ < opts_.frame_gap_ ) return false;   // 刚开始建图
    if ( current_frame_->keyframe_id_ - last_loop_closure_kf_id_ < opts_.loop_closure_gap_ ) return false;  // 距离上次回环太近
    
    Vec2d current_frame_t = current_frame_->pose_.translation();
    for ( auto & frame : frames_ ) {
        /// 1.跳过最近的帧
        if ( ( current_frame_->keyframe_id_ - frame.first ) <= opts_.frame_gap_ ) continue;

        // 2.排除已经存在回环约束的帧对
        auto hist_iter = loop_constraints_.find( std::pair<size_t, size_t>(frame.first, current_frame_->keyframe_id_) );
        if ( hist_iter != loop_constraints_.end() && hist_iter->second.valid_ ) continue;

        // 3.检查位置距离是否小于阈值
        Vec2d frame_t = frame.second->pose_.translation();
        double dis = ( frame_t - current_frame_t ).norm();
        if ( dis < opts_.candidate_distance_th_ ) {
            // LOG(INFO) << "Check frame " << current_frame_->keyframe_id_ << " with frame " << frame.first << ", distance: " << dis << "m";
            current_candidates_.emplace_back(frame.first);
        }
    }
    return !current_candidates_.empty();
}

void LoopClosure::match(){
    start_kf_id_ = opts_.enable_global_optimization_ ? 0 : UINT32_MAX;  // 如果启动全局优化，那就直接设为 0
    for ( size_t & can : current_candidates_ ) {
        auto can_frame = frames_[can];

        /// TUDO 几何验证
        /// ...

        // 相对约束
        // l1: 历史帧 candidate
        // l2: 当前帧 current
        // 传入: T_l1_l2
        // e = Log( T_l1_w * T_w_l2 * T_l1_l2.inv )
        SE2 T_l1_l2;
        if ( matchFrameWithNDT( can_frame, T_l1_l2 ) ) {
            loop_constraints_.emplace(
                std::pair<size_t, size_t>(can, current_frame_->keyframe_id_),
                LoopConstraint(can, current_frame_->keyframe_id_, T_l1_l2));
            has_new_loops_ = true;
            start_kf_id_ = ( can < start_kf_id_ ) ? can : start_kf_id_;
            debug_fout << "kf & can:" << current_frame_->keyframe_id_ << " " << current_frame_->pose_.log().transpose() << " " << can << " " << can_frame->pose_.log().transpose() << std::endl;
        }
    }
    current_candidates_.clear();
}

void LoopClosure::optimize(){
    loop_success_ = false;
    using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<3,1>>;
    using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;
    auto linearSolver = g2o::make_unique<LinearSolverType>();
    auto blockSolver = g2o::make_unique<BlockSolverType>(std::move(linearSolver));
    auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // 顶点：每个帧的位姿
    LOG(INFO) << "Loop: " << start_kf_id_ << " -> " << current_frame_->keyframe_id_;
    for (int i = start_kf_id_; i <= current_frame_->keyframe_id_; ++ i) {
        /// 优化回环以内的帧
        auto v = new VertexSE2();
        v->setId(i);
        v->setEstimate(frames_[i]->pose_);
        optimizer.addVertex(v);
    }

    // 固定第一帧, 防止过度扭曲, 防止出现断裂的轨迹
    optimizer.vertex(start_kf_id_)->setFixed(true);

    // 1.连续约束  只优化最小候选帧到当前帧之前的帧
    for (int i = start_kf_id_; i < current_frame_->keyframe_id_; ++ i) {
        SE2 pose1   = frames_[i]->pose_;
        SE2 pose2   = frames_[i+1]->pose_;
        SE2 T_l1_l2 = pose1.inverse() * pose2;

        EdgeSE2 * e = new EdgeSE2();
        e->setVertex(0, optimizer.vertex(i));
        e->setVertex(1, optimizer.vertex(i + 1));
        // l1: 历史帧 first
        // l2: 当前帧 second
        // 传入: T_l1_l2
        // e = Log( T_l1_w * T_w_l2 * T_l1_l2.inv )
        e->setMeasurement( std::move(T_l1_l2) );
        e->setInformation( Mat3d::Identity() * opts_.all_constraints_info_weight_ );  // 高信息矩阵（强约束） 加大这个权重, 回环效果好了一点
        optimizer.addEdge(e);
    }

    // 2.回环约束
    std::map<std::pair<size_t, size_t>, EdgeSE2 *> loop_edges;
    for ( auto & lc : loop_constraints_ ) {
        if ( !lc.second.valid_ ) continue;
        if ( lc.first.first < start_kf_id_ ) continue;  // 不优化 start_kf_id_ 之前的回环

        auto frame1 = frames_[lc.first.first];
        auto frame2 = frames_[lc.first.second];

        EdgeSE2 * e = new EdgeSE2();
        e->setVertex(0, optimizer.vertex(frame1->keyframe_id_)); // T_w_l1
        e->setVertex(1, optimizer.vertex(frame2->keyframe_id_)); // T_w_l2
        e->setMeasurement(lc.second.T12_);  // 检测到的相对位姿
        e->setInformation( Mat3d::Identity() * opts_.loop_constraints_info_weight_ );

        auto rk = new g2o::RobustKernelCauchy;
        rk->setDelta(opts_.loop_rk_delta_);
        e->setRobustKernel(rk);
        optimizer.addEdge(e);
        loop_edges.emplace(lc.first, e);
    }

    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(opts_.max_opti_iter_);  // 第一次优化

    int inliers = 0;
    for ( auto & ep : loop_edges ) {
        // 检查回环约束的残差（chi2）
        if ( ep.second->chi2() < opts_.loop_rk_delta_ ) {
            // 有效回环，移除鲁棒核函数（设为内点）
            ep.second->setRobustKernel(nullptr);
            loop_constraints_.at(ep.first).valid_ = true;
            ++inliers;
        } else {
            // 无效回环，标记为边缘化
            ep.second->setLevel(1);
            loop_constraints_.at(ep.first).valid_ = false;
        }
    }

    optimizer.optimize(opts_.max_opti_iter_/2);

    last_loop_closure_kf_id_ = current_frame_->keyframe_id_;
    LOG(INFO) << "Key frame: " << current_frame_->keyframe_id_ << " -> loop inliers: " << inliers << "/" << loop_constraints_.size();

    //   这个地方总会报 segment fault
    // 1.先求新帧的相对位姿, 后面更新完之后再用相对位姿更新这些新的帧
    frames_ = map_->getAllFrames();
    last_update_kf_id_ = frames_.size() - 1;   // 给前端更新当前普通帧用的
    int add_new_num_frame = last_update_kf_id_ - current_frame_->keyframe_id_;  // -1因为 id 从 0 开始
    std::vector<SE2> T12;
    T12.reserve(add_new_num_frame);
    for ( size_t i = 0; i < add_new_num_frame; ++i ) {
        size_t id1 = current_frame_->keyframe_id_ + i;
        size_t id2 = id1 + 1;
        SE2 T_w_l1 = map_->getKf(id1)->pose_;  // 从优化时的 current frame 开始
        SE2 T_w_l2 = map_->getKf(id2)->pose_;
        SE2 T_l1_l2 = T_w_l1.inverse() * T_w_l2;    // T12 = T_l1_w * T_w_l2
        T12.emplace_back( std::move(T_l1_l2) );
    }
    /// 更新回环以内的帧  这里用 <= , current_frame_也要更新
    for (size_t i = start_kf_id_; i <= current_frame_->keyframe_id_; ++ i) {
        VertexSE2 * v = ( VertexSE2 * ) optimizer.vertex( i );
        map_->updateKf(i, std::move(v->estimate()));
    }
    // 更新完之后，再用相对位姿更新前端新添的帧
    for ( size_t i = 0; i < add_new_num_frame; ++i ) {
        size_t id1 = current_frame_->keyframe_id_ + i;
        size_t id2 = id1 + 1;
        SE2 new_pose = map_->getKf(id1)->pose_ * T12[i]; // T_w_l2 = T_w_l1 * T_l1_l2
        map_->updateKf(id2, std::move(new_pose));
    }

    loop_success_ = true;

    // 2.后端线程更新 occupancy map
    // std::thread([&]() {
    //     map_->getOccupancyMap().addLidarFrameInBuffer( frames_, opts_.num_frame_gap_to_new_occu_map_);
    // }).detach();

    // 3.更新完位姿后再更新 ndt
    int num_frames = frames_.size() >= opts_.num_frames_add_in_new_ndt_ ? opts_.num_frames_add_in_new_ndt_ : frames_.size();
    std::vector<std::shared_ptr<Frame>> frames_add_in_new_ndt;
    frames_add_in_new_ndt.reserve(num_frames);
    auto riter = frames_.rbegin();  // rbegin 指向最后一个元素
    for (int i = 0; i < num_frames && riter != frames_.rend(); ++i, ++riter){
        frames_add_in_new_ndt.emplace_back(riter->second);
    }
    map_->getNdt().addScanInGridsBuffer( std::move(frames_add_in_new_ndt) );

    // 4.移除错误的匹配
    for ( auto iter = loop_constraints_.begin(); iter != loop_constraints_.end(); ) {
        if ( !iter->second.valid_ ) {
            iter = loop_constraints_.erase(iter);
        } else {
            ++iter;
        }
    }

    // 5.最后更新 occupancy map 如果开后台更新, 有可能导致更新太慢, 然后再次调用发生 segment default
    map_->getOccupancyMap().addLidarFrameInBuffer( map_->getAllFrames(), opts_.num_frame_gap_to_new_occu_map_);
}


bool LoopClosure::matchFrameWithNDT( std::shared_ptr<Frame> candidate_frame, SE2 & T_l1_l2 ){
    // 创建局部 NDT 进行匹配
    NdtInc2d::Options ndt_opts(map_->getNdtOptions());

    // SE2 init_T_l1_l2 = candidate_frame->pose_.inverse() * current_frame_->pose_;
    SE2 init_pose = current_frame_->pose_;  // 将当前帧作为初始位姿
    int score = 0;
    for ( float & r : opts_.multi_ndt_resolution_) {
        ndt_opts.voxel_size_ = r;
        NdtInc2d local_ndt(ndt_opts);
        // addScan 里面已经将体素转到直接坐标系了
        // 这样得到的是 T_l1_l2
        local_ndt.setSource( current_frame_ );       // 当前帧与局部地图匹配
        for ( int i = opts_.left_can_id_; i < opts_.right_can_id_; ++i ) {  // 将 candidate 附近的 frame 也放进 local ndt 中
            int id = candidate_frame->keyframe_id_ + i;
            if ( id < 0 ) continue;  // 因为帧触发的间隔, 不会大于 frames_ 的数量
            local_ndt.addScan( frames_[id] );        // 候选帧构建局部地图
        }
        if ( !local_ndt.alignNdt( init_pose ) ) break;  // 不符合就提前退出
        ++score;
    }
    
    if ( score < opts_.multi_ndt_resolution_.size() ) {
        // LOG(WARNING) << "NDT match failed, score: " << score << "/" << opts_.multi_ndt_resolution_.size();
        return false;
    }
    T_l1_l2 = candidate_frame->pose_.inverse() * init_pose;
    return true;
}

void LoopClosure::loop(){
    while ( thread_running_.load() ) {
        if ( !detect() ) continue;

        match();

        if ( has_new_loops_ ) optimize();
    }
}

}

