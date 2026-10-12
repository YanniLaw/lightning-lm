/*
 * Copyright 2016 The Cartographer Authors
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

#ifndef SRC_UTILS_THREAD_POOL_H_
#define SRC_UTILS_THREAD_POOL_H_

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace lightning {

class Task;

class ThreadPoolInterface {
   public:
    ThreadPoolInterface() = default;
    virtual ~ThreadPoolInterface() = default;

    virtual std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) = 0;

   protected:
    void Execute(Task* task);
    void SetThreadPool(Task* task);

   private:
    friend class Task;

    virtual void NotifyDependenciesCompleted(Task* task) = 0;
};

// A fixed-size task executor. The pool must be destroyed after all tasks have
// completed; its destructor then joins all worker threads.
class ThreadPool : public ThreadPoolInterface {
   public:
    explicit ThreadPool(int num_threads);
    ~ThreadPool() override;

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // An expired handle means the task completed and no longer blocks dependents.
    std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) override;

   private:
    void DoWork();
    void NotifyDependenciesCompleted(Task* task) override;

    std::mutex mutex_;
    std::condition_variable condition_;

    // Protected by mutex_.
    bool running_ = true;
    std::vector<std::thread> workers_;
    std::deque<std::shared_ptr<Task>> task_queue_;
    std::unordered_map<Task*, std::shared_ptr<Task>> tasks_not_ready_;
};

}  // namespace lightning

#endif  // SRC_UTILS_THREAD_POOL_H_
