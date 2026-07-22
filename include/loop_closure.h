#ifndef __LOOP_CLOSURE_H
#define __LOOP_CLOSURE_H

#include "2dNdtLIO/common/eigen_types.h"
#include "2dNdtLIO/include/map.h"

#include <memory>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <fstream>

namespace sad {

class LoopClosure {
public:
    struct Options {
        std::string debug_fout_ = "../map/loop.txt";
        bool enable_global_optimization_ = true;
        int left_can_id_ = -3;
        int right_can_id_ = 3;
        int max_opti_iter_ = 10;
        float candidate_distance_th_ = 2.0;          // candidate frame 与 frame 之间的距离
        int loop_closure_gap_ = 10;                  // 要隔至少 x 关键帧再回环, 要考虑更新占据图
        int frame_gap_ = 100;                        // current frame 与历史帧的间隔
        float loop_rk_delta_ = 1.0;                  // loop closure 的 robust kernel 阈值
        double all_constraints_info_weight_ = 1e4;   // 如果位姿校得太过了，那就是这个太小了
        double loop_constraints_info_weight_ = 1.0;
        int num_frame_gap_to_new_occu_map_ = 5;      // 回环之后 隔 x 关键帧放一个进 occupancy map
        int num_frames_add_in_new_ndt_ = 10;         // 回环之后新添加进地图的帧数量
        std::vector<float> multi_ndt_resolution_ = {3.0, 2.5, 2.0, 1.5, 1.0, 0.5};
    };

    /// 一个回环约束
    struct LoopConstraint {
        LoopConstraint( size_t & id_frame1, size_t & id_frame2, SE2 & T12 )
        : id_frame1_(id_frame1), id_frame2_(id_frame2), T12_(T12) {}
        size_t id_frame1_ = 0;
        size_t id_frame2_ = 0;
        SE2 T12_;
        bool valid_ = true;
    };
    
    LoopClosure( Options opts );

    void setMap( std::shared_ptr<Map> map ) { map_ = map; }
    void addNewFrame( std::shared_ptr<Frame> frame );
    bool loopSuccess( size_t & last_update_kf_id ) {
        if ( !loop_success_ ) return false;
        last_update_kf_id = last_update_kf_id_;
        bool temp = loop_success_;
        loop_success_ = false;
        return temp;
    }
    void close();
private:
    bool detect();
    void match();
    void optimize();
    
    /**
     * T_l1_l2: 1旧的帧, 2较新的帧
     */
    bool matchFrameWithNDT( std::shared_ptr<Frame> candidate_frame, SE2 & T_l1_l2 );

    void loop();
private:
    Options opts_;

    std::ofstream debug_fout;

    // 线程相关
    std::mutex data_mutex_;
    std::thread thread_;
    std::atomic<bool> thread_running_{false};
    std::condition_variable thread_step_;

    // 地图
    std::shared_ptr<Map> map_ = nullptr;

    // current_frame 是后面优化的时候使用 | current_add_frame 是前端送来的当前帧
    // current_frame 只在 DetectLoopCandidates 开头的锁读取 current_add_frame
    // 否则在后面优化的时候，current_frame 可能会因为 AddNewFrame 而改变
    int last_loop_closure_kf_id_ = 0;     // 上一个回环检测的 kf 的 id, 必须在优化完之后更新
    size_t last_update_kf_id_ = 0;        // 给前端更新当前普通帧用的
    size_t start_kf_id_ = UINT32_MAX;
    bool has_new_loops_ = false;
    bool has_new_frame_ = false;
    std::atomic<bool> loop_success_ = false;
    std::shared_ptr<Frame> current_frame_ = nullptr;
    std::shared_ptr<Frame> current_add_frame_ = nullptr;   // AddNewFrame 中使用

    std::map<size_t, std::shared_ptr<Frame>> frames_;
    std::vector<size_t> current_candidates_;       // 可能存在的回环
    std::map<std::pair<size_t, size_t>, LoopConstraint> loop_constraints_;  // 成功的回环约束

};

}



#endif // __LOOP_CLOSURE_H
