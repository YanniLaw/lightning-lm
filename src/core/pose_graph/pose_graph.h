// Copyright 2026
//
// Pose graph maintenance and optimization.

#ifndef SRC_CORE_POSE_GRAPH_POSE_GRAPH_H_
#define SRC_CORE_POSE_GRAPH_POSE_GRAPH_H_

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "common/eigen_types.h"
#include "common/keyframe.h"
#include "common/pose_graph_data.h"
#include "common/wheel_odometry_data.h"
#include "core/lio/lio.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/keyframe_map.h"
#include "core/optimization/graph_optimizer.h"
#include "core/pose_graph/keyframe_collection.h"
#include "core/pose_graph/work_queue.h"

namespace lightning {

class ThreadPool;

// Owns the serial backend pipeline, loop-closure constraints, and graph optimizer.
// Internal backend failures terminate the process.
class PoseGraph {
   public:
    struct Options {
        Options() {}

        bool verbose_ = true;       // 输出调试信息
        bool enable_loop_closing_ = true;       // 是否启用回环检测
        std::string optimizer_type_ = "miao";   // 优化器类型

        int detection_interval_ = 20;       // 每隔多少个关键帧检查一次
        int optimize_every_n_nodes_ = 20;   // 每隔多少个节点进行一次优化
        int num_threads_ = 2;               // 后端线程池线程数
        int min_id_interval_ = 20;          // 被检查的关键帧ID间隔
        int closest_id_th_ = 50;            // 历史关键帧与当前帧的ID间隔
        double max_range_ = 30.0;           // 候选帧的最大距离
        double ndt_score_th_ = 1.0;         // ndt位姿分值
        /// 图优化权重
        double motion_trans_noise_ = 0.1;               // 运动模型平移噪声
        double motion_rot_noise_ = 3.0 * M_PI / 180.0;  // 运动模型旋转噪声
        double loop_trans_noise_ = 0.2;                 // 回环平移噪声
        double loop_rot_noise_ = 3.0 * M_PI / 180.0;    // 回环旋转噪声
        double rk_loop_th_ = 5.2 / 5;                   // 回环检测阈值 (距离阈值)

        bool with_height_ = true;       // 是否考虑高度信息
        double height_noise_ = 0.1;     // 高度噪声
    };

    explicit PoseGraph(Options options = Options());
    ~PoseGraph();

    PoseGraph(const PoseGraph&) = delete;
    PoseGraph& operator=(const PoseGraph&) = delete;

    bool Init(const std::string& yaml_path);

    // Accepts a scan-matched frontend result and assigns the backend ID. The
    // result is queued and processed by the unified backend work queue.
    Keyframe::Ptr AddKeyframe(const LIOResult& result);

    // Wheel input is reserved for future graph constraints.
    LIOInputStatus AddWheelOdometry(const WheelOdometryData& data);

    // Returns a snapshot of the currently committed graph state. Use
    // FlushAndGetMapSnapshot before saving when all queued work must be done.
    std::vector<KeyframeMapEntry> GetMapSnapshot() const;

    // Completes all work accepted before the call and returns a consistent map
    // snapshot. Calls are serialized with one another.
    bool FlushAndGetMapSnapshot(std::vector<KeyframeMapEntry>* snapshot);

    // Stops accepting new keyframes, flushes the backend, and joins workers.
    // The operation is idempotent.
    void Stop();

    using OptimizedCallback = std::function<void()>;
    void SetOptimizedCallback(OptimizedCallback callback) {
        optimized_callback_ = std::move(callback);
    }

    using DataCallback = std::function<void(PoseGraphDataPtr)>;
    void SetDataCallback(DataCallback callback) { data_callback_ = std::move(callback); }

   private:
    struct FlushState {
        std::mutex mutex;
        std::condition_variable condition;
        bool done = false;
        std::vector<KeyframeMapEntry> snapshot;
    };

    void AddWorkItem(std::function<WorkItem::Result()> task);
    void DrainWorkQueue();

    WorkItem::Result HandleKeyframe(const Keyframe::Ptr& keyframe);
    void HandleWorkQueue(LoopClosing::MatchResult result);
    void CompleteFlush(const std::shared_ptr<FlushState>& flush_state,
                       std::vector<KeyframeMapEntry> snapshot);

    bool AddKeyframeToGraph(const Keyframe::Ptr& current);
    std::size_t AddLoopConstraints(const std::vector<LoopCandidate>& constraints);
    bool OptimizeGraph();

    // Caller holds graph_mutex_.
    PoseGraphDataPtr CreateDataSnapshot(bool optimized);

    Options options_;
    bool initialized_ = false;
    std::atomic_bool accepting_{false};
    std::atomic_bool stopped_{false};
    std::mutex input_mutex_;
    // Const snapshot reads also need to synchronize with graph updates.
    mutable std::mutex graph_mutex_;
    std::mutex flush_mutex_;
    std::mutex stop_mutex_;
    std::condition_variable stop_condition_;
    bool stop_in_progress_ = false;

    std::uint64_t next_keyframe_id_ = 0;
    double last_accepted_timestamp_ = -1.0;

    KeyframeCollection keyframes_;
    LoopClosing loop_closing_;
    std::unique_ptr<ThreadPool> thread_pool_;

    std::mutex work_queue_mutex_;
    std::unique_ptr<WorkQueue> work_queue_;

    std::size_t unoptimized_nodes_ = 0;
    bool graph_dirty_ = false;

    OptimizedCallback optimized_callback_;
    DataCallback data_callback_;
    std::uint64_t data_version_ = 0;

    std::unique_ptr<GraphOptimizer> optimizer_;

    Mat6d info_motion_ = Mat6d::Identity();
    Mat6d info_loops_ = Mat6d::Identity();

    std::uint64_t next_constraint_id_ = 1;
    std::vector<PoseGraphConstraint> loop_constraints_;
    std::set<std::pair<std::uint64_t, std::uint64_t>> loop_pairs_;
};

}  // namespace lightning

#endif  // SRC_CORE_POSE_GRAPH_POSE_GRAPH_H_
