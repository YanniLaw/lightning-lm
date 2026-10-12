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

#include "utils/thread_pool.h"

#include <glog/logging.h>

#include <cerrno>
#include <cstddef>

#ifdef __linux__
#include <unistd.h>
#endif

#include "utils/task.h"

namespace lightning {

void ThreadPoolInterface::Execute(Task* task) { task->Execute(); }

void ThreadPoolInterface::SetThreadPool(Task* task) { task->SetThreadPool(this); }

ThreadPool::ThreadPool(int num_threads) {
    CHECK_GT(num_threads, 0) << "ThreadPool requires at least one worker.";
    workers_.reserve(static_cast<std::size_t>(num_threads));
    for (int i = 0; i < num_threads; ++i) {
        workers_.emplace_back([this]() { DoWork(); });
    }
}

ThreadPool::~ThreadPool() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        CHECK(running_);
        running_ = false;
    }
    condition_.notify_all();

    for (std::thread& worker : workers_) {
        worker.join();
    }
}

void ThreadPool::NotifyDependenciesCompleted(Task* task) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = tasks_not_ready_.find(task);
        CHECK(it != tasks_not_ready_.end());
        task_queue_.push_back(it->second);
        tasks_not_ready_.erase(it);
    }
    condition_.notify_one();
}

std::weak_ptr<Task> ThreadPool::Schedule(std::unique_ptr<Task> task) {
    CHECK(task != nullptr);

    std::shared_ptr<Task> shared_task;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        Task* const task_pointer = task.get();
        const auto insert_result =
            tasks_not_ready_.emplace(task_pointer, std::move(task));
        CHECK(insert_result.second) << "Schedule called twice for the same task.";
        shared_task = insert_result.first->second;
    }

    SetThreadPool(shared_task.get());
    return shared_task;
}

void ThreadPool::DoWork() {
#ifdef __linux__
    errno = 0;
    if (nice(10) == -1 && errno != 0) {
        PLOG(WARNING) << "Failed to lower thread-pool worker priority.";
    }
#endif

    while (true) {
        std::shared_ptr<Task> task;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this]() {
                return !task_queue_.empty() || !running_;
            });
            if (!task_queue_.empty()) {
                task = std::move(task_queue_.front());
                task_queue_.pop_front();
            } else if (!running_) {
                return;
            }
        }

        CHECK(task != nullptr);
        CHECK(task->GetState() == Task::State::kDependenciesCompleted);
        Execute(task.get());
    }
}

}  // namespace lightning
