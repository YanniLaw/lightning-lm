// Copyright 2026
//
// Loop candidate detection and constraint construction.

#ifndef SRC_CORE_LOOP_CLOSING_LOOP_CLOSING_H_
#define SRC_CORE_LOOP_CLOSING_LOOP_CLOSING_H_

#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "core/loop_closing/loop_match_data.h"
#include "utils/task.h"

namespace lightning {

class KeyframeCollection;
class ThreadPoolInterface;

// Finds candidate loop closures and computes their relative pose constraints.
class LoopClosing {
   public:
    struct MatchResult {
        std::vector<LoopCandidate> constraints;
    };

    using DoneCallback = std::function<void(MatchResult)>;

    struct Options {
        Options() {}

        bool verbose_ = true;

        int detection_interval_ = 20;
        int min_id_interval_ = 20;
        int closest_id_th_ = 50;
        double max_range_ = 30.0;
        double ndt_score_th_ = 1.0;
    };

    explicit LoopClosing(Options options = Options());
    ~LoopClosing();

    LoopClosing(const LoopClosing&) = delete;
    LoopClosing& operator=(const LoopClosing&) = delete;

    // Applies configuration before the first constraint computation.
    void SetOptions(Options options);

    // Computes valid loop constraints synchronously for one keyframe.
    std::vector<LoopCandidate> ComputeConstraints(const Keyframe::Ptr& current,
                                                  const KeyframeCollection& keyframes);

    // Captures all mutable keyframe state needed by independent match tasks.
    std::vector<LoopMatchInput> PrepareMatches(const Keyframe::Ptr& current,
                                               const KeyframeCollection& keyframes);

    // Computes one match without accessing the keyframe collection.
    std::optional<LoopCandidate> ComputeMatch(const LoopMatchInput& input) const;

    // Sets the executor used for asynchronous loop matching. The executor is
    // owned by PoseGraph and must outlive this object and all scheduled tasks.
    void SetThreadPool(ThreadPoolInterface* thread_pool);

    // Schedules the match tasks for the current node.
    void ScheduleMatches(std::vector<LoopMatchInput> inputs);

    // Finishes submitting a node's match tasks. Call after ScheduleMatches,
    // including when the node has no match candidates.
    void NotifyEndOfNode();

    // Returns the number of nodes whose matching tasks have completed.
    int GetNumFinishedNodes() const;

    // Returns the number of nodes submitted for matching but not yet completed.
    int GetNumPendingNodes() const;

    // Completes asynchronously after all previously scheduled matches finish.
    void WhenDone(DoneCallback callback);

   private:
    std::vector<LoopCandidate> DetectLoopCandidates(const Keyframe::Ptr& current,
                                                    const KeyframeCollection& keyframes);

    std::optional<LoopMatchInput> BuildMatchInput(const KeyframeCollection& keyframes,
                                                  const LoopCandidate& candidate) const;

    void RunWhenDoneCallback();

    Options options_;
    Keyframe::Ptr last_loop_kf_;
    int keyframes_since_detection_ = 0;

    mutable std::mutex match_mutex_;
    ThreadPoolInterface* thread_pool_ = nullptr;
    DoneCallback done_callback_;
    std::unique_ptr<Task> finish_node_task_ = std::make_unique<Task>();
    std::unique_ptr<Task> when_done_task_ = std::make_unique<Task>();
    std::deque<std::optional<LoopCandidate>> match_results_;
    // Protected by match_mutex_.
    int num_started_nodes_ = 0;
    int num_finished_nodes_ = 0;
};

}  // namespace lightning

#endif  // SRC_CORE_LOOP_CLOSING_LOOP_CLOSING_H_
