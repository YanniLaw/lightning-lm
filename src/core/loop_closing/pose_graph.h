// Copyright 2026
//
// Pose graph maintenance and optimization.

#ifndef LIGHTNING_POSE_GRAPH_H
#define LIGHTNING_POSE_GRAPH_H

#include <atomic>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/eigen_types.h"
#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "core/loop_closing/keyframe_collection.h"
#include "core/loop_closing/loop_closing.h"
#include "utils/async_message_process.h"

namespace lightning::miao {
class EdgeSE3;
class Optimizer;
class VertexSE3;
}  // namespace lightning::miao

namespace lightning {

// Owns the serial backend pipeline, loop-closure constraints, and graph optimizer.
class PoseGraph {
   public:
    struct Options {
        Options() {}

        bool verbose_ = true;       // 输出调试信息
        bool online_mode_ = false;  // 切换离线-在线模式

        int loop_kf_gap_ = 20;          // 每隔多少个关键帧检查一次
        int min_id_interval_ = 20;      // 被检查的关键帧ID间隔
        int closest_id_th_ = 50;        // 历史关键帧与当前帧的ID间隔
        double max_range_ = 30.0;       // 候选帧的最大距离
        double ndt_score_th_ = 1.0;     // ndt位姿分值

        /// 图优化权重
        double motion_trans_noise_ = 0.1;               // 位移权重
        double motion_rot_noise_ = 3.0 * M_PI / 180.0;  // 旋转权重

        double loop_trans_noise_ = 0.2;                 // 位移权重
        double loop_rot_noise_ = 3.0 * M_PI / 180.0;    // 旋转权重

        double rk_loop_th_ = 5.2 / 5;   // 回环的RK阈值

        bool with_height_ = true;
        double height_noise_ = 0.1;
    };

    explicit PoseGraph(Options options = Options());
    ~PoseGraph();

    PoseGraph(const PoseGraph&) = delete;
    PoseGraph& operator=(const PoseGraph&) = delete;

    void Init(const std::string& yaml_path);

    // Queues a keyframe for online processing or handles it synchronously in
    // offline mode.
    void AddKeyframe(Keyframe::Ptr keyframe);

    // Stops accepting new keyframes and drains the online queue.
    void Stop();

    using OptimizedCallback = std::function<void()>;
    void SetOptimizedCallback(OptimizedCallback callback) { optimized_callback_ = std::move(callback); }

   private:
    void HandleKeyframe(const Keyframe::Ptr& keyframe);

    // Adds one graph node and its constraints, then optimizes when a valid
    // loop constraint is present. Returns true after pose write-back.
    bool UpdateGraphAndOptimize(const Keyframe::Ptr& current,
                                const std::vector<LoopCandidate>& constraints);

    Options options_;
    bool initialized_ = false;
    std::atomic_bool accepting_{false};

    KeyframeCollection keyframes_;
    LoopClosing loop_closing_;
    AsyncMessageProcess<Keyframe::Ptr> keyframe_thread_;

    Keyframe::Ptr last_keyframe_;
    OptimizedCallback optimized_callback_;

    std::shared_ptr<miao::Optimizer> optimizer_;

    Mat6d info_motion_ = Mat6d::Identity();
    Mat6d info_loops_ = Mat6d::Identity();

    std::vector<std::shared_ptr<miao::VertexSE3>> keyframe_vertices_;
    std::vector<std::shared_ptr<miao::EdgeSE3>> loop_edges_;
};

}  // namespace lightning

#endif  // LIGHTNING_POSE_GRAPH_H
