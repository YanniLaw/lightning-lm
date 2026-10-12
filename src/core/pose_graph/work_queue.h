// Copyright 2026
//
// Serial pose graph work items.

#ifndef SRC_CORE_POSE_GRAPH_WORK_QUEUE_H_
#define SRC_CORE_POSE_GRAPH_WORK_QUEUE_H_

#include <chrono>
#include <deque>
#include <functional>

namespace lightning {

struct WorkItem {
    enum class Result { kDoNotRunOptimization, kRunOptimization };

    std::chrono::steady_clock::time_point time;
    std::function<Result()> task;
};

using WorkQueue = std::deque<WorkItem>;

}  // namespace lightning

#endif  // SRC_CORE_POSE_GRAPH_WORK_QUEUE_H_
