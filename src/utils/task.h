/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// Adapted for Lightning in 2026: namespace, naming, and project integration.

#ifndef SRC_UTILS_TASK_H_
#define SRC_UTILS_TASK_H_

#include <functional>
#include <memory>
#include <mutex>
#include <set>

namespace lightning {

class ThreadPoolInterface;

// A task runs after all tasks it depends on have completed.
class Task {
   public:
    friend class ThreadPoolInterface;

    using WorkItem = std::function<void()>;

    enum class State { kNew, kDispatched, kDependenciesCompleted, kRunning, kCompleted };

    Task() = default;
    ~Task();

    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;

    State GetState();

    // The task must still be in the NEW state.
    void SetWorkItem(const WorkItem& work_item);

    // The task must still be in the NEW state. An expired dependency has
    // already completed and therefore does not block this task.
    void AddDependency(std::weak_ptr<Task> dependency);

   private:
    void AddDependentTask(Task* dependent_task);
    void Execute();
    void SetThreadPool(ThreadPoolInterface* thread_pool);
    void OnDependencyCompleted();

    // Protected by mutex_.
    WorkItem work_item_;
    ThreadPoolInterface* thread_pool_to_notify_ = nullptr;
    State state_ = State::kNew;
    unsigned int uncompleted_dependencies_ = 0;
    std::set<Task*> dependent_tasks_;

    std::mutex mutex_;
};

}  // namespace lightning

#endif  // SRC_UTILS_TASK_H_
