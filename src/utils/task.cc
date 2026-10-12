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

#include "utils/task.h"

#include <mutex>

#include <glog/logging.h>

#include "utils/thread_pool.h"

namespace lightning {

Task::~Task() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::kNew && state_ != State::kCompleted) {
        LOG(WARNING) << "Deleting a task before it completed.";
    }
}

Task::State Task::GetState() {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void Task::SetWorkItem(const WorkItem& work_item) {
    std::lock_guard<std::mutex> lock(mutex_);
    CHECK(state_ == State::kNew);
    work_item_ = work_item;
}

void Task::AddDependency(std::weak_ptr<Task> dependency) {
    std::shared_ptr<Task> shared_dependency;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        CHECK(state_ == State::kNew);
        shared_dependency = dependency.lock();
        if (shared_dependency) {
            ++uncompleted_dependencies_;
        }
    }

    if (shared_dependency) {
        shared_dependency->AddDependentTask(this);
    }
}

void Task::AddDependentTask(Task* dependent_task) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == State::kCompleted) {
        dependent_task->OnDependencyCompleted();
        return;
    }

    const bool inserted = dependent_tasks_.insert(dependent_task).second;
    CHECK(inserted) << "Task was registered as a dependency more than once.";
}

void Task::OnDependencyCompleted() {
    std::lock_guard<std::mutex> lock(mutex_);
    CHECK(state_ == State::kNew || state_ == State::kDispatched);
    CHECK_GT(uncompleted_dependencies_, 0U);
    --uncompleted_dependencies_;
    if (uncompleted_dependencies_ == 0U && state_ == State::kDispatched) {
        state_ = State::kDependenciesCompleted;
        CHECK(thread_pool_to_notify_ != nullptr);
        thread_pool_to_notify_->NotifyDependenciesCompleted(this);
    }
}

void Task::SetThreadPool(ThreadPoolInterface* thread_pool) {
    std::lock_guard<std::mutex> lock(mutex_);
    CHECK(state_ == State::kNew);
    state_ = State::kDispatched;
    thread_pool_to_notify_ = thread_pool;
    if (uncompleted_dependencies_ == 0U) {
        state_ = State::kDependenciesCompleted;
        CHECK(thread_pool_to_notify_ != nullptr);
        thread_pool_to_notify_->NotifyDependenciesCompleted(this);
    }
}

void Task::Execute() {
    WorkItem work_item;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        CHECK(state_ == State::kDependenciesCompleted);
        state_ = State::kRunning;
        work_item = work_item_;
    }

    if (work_item) {
        work_item();
    }

    std::set<Task*> dependent_tasks;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        CHECK(state_ == State::kRunning);
        state_ = State::kCompleted;
        dependent_tasks = std::move(dependent_tasks_);
    }

    for (Task* dependent_task : dependent_tasks) {
        dependent_task->OnDependencyCompleted();
    }
}

}  // namespace lightning
