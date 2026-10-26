#include "core/system/sensor_dispatcher.h"

#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "core/lightning_math.hpp"

namespace lightning::sys {
namespace {

bool PointCloudBytes(const CloudPtr& scan, std::size_t& bytes) {
    if (scan->points.capacity() > std::numeric_limits<std::size_t>::max() / sizeof(PointType)) {
        return false;
    }
    bytes = scan->points.capacity() * sizeof(PointType);
    return true;
}

}  // namespace

SensorDispatcher::SensorDispatcher(Options options, ImuCallback imu_callback,
                                   ScanCallback scan_callback,
                                   WheelCallback wheel_callback)
    : options_(options),
      imu_callback_(std::move(imu_callback)),
      scan_callback_(std::move(scan_callback)),
      wheel_callback_(std::move(wheel_callback)) {}

SensorDispatcher::~SensorDispatcher() { Stop(StopMode::kCancel); }

void SensorDispatcher::Start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (started_) {
        return;
    }

    stop_requested_ = false;
    drain_on_stop_ = true;
    faulted_ = false;
    started_ = true;
    worker_ = std::thread(&SensorDispatcher::Run, this);
}

InputResult SensorDispatcher::AddImu(IMUPtr imu) {
    if (!IsValidImu(imu)) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.rejected_invalid;
        return InputResult::InvalidData;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_ || stop_requested_) {
        return InputResult::NotRunning;
    }
    if (faulted_ || imu_queue_.size() >= options_.max_imu_queue_size) {
        faulted_ = true;
        ++stats_.rejected_queue_full;
        return InputResult::QueueFull;
    }
    if (last_accepted_imu_time_ >= 0 &&
        imu->timestamp < last_accepted_imu_time_) {
        ++stats_.rejected_time;
        return InputResult::TimeDiscontinuity;
    }

    last_accepted_imu_time_ = imu->timestamp;
    imu_queue_.emplace_back(std::move(imu));
    ++stats_.accepted_imu;
    stats_.peak_imu_queue_size = std::max(stats_.peak_imu_queue_size, imu_queue_.size());
    condition_.notify_one();
    return InputResult::Accepted;
}

InputResult SensorDispatcher::AddPointCloud(CloudPtr scan) {
    if (!IsValidScan(scan)) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.rejected_invalid;
        return InputResult::InvalidData;
    }

    std::size_t bytes = 0;
    if (!PointCloudBytes(scan, bytes) || bytes > options_.max_point_cloud_bytes) {
        std::lock_guard<std::mutex> lock(mutex_);
        faulted_ = true;
        ++stats_.rejected_queue_full;
        return InputResult::QueueFull;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_ || stop_requested_) {
        return InputResult::NotRunning;
    }
    if (faulted_ || scan_queue_.size() >= options_.max_scan_queue_size ||
        bytes > options_.max_point_cloud_bytes - queued_point_cloud_bytes_) {
        faulted_ = true;
        ++stats_.rejected_queue_full;
        return InputResult::QueueFull;
    }

    const double timestamp = math::ToSec(scan->header.stamp);
    if (last_accepted_scan_time_ >= 0 && timestamp < last_accepted_scan_time_) {
        ++stats_.rejected_time;
        return InputResult::TimeDiscontinuity;
    }

    PendingScan pending;
    pending.data = std::move(scan);
    pending.bytes = bytes;
    last_accepted_scan_time_ = timestamp;
    queued_point_cloud_bytes_ += bytes;
    scan_queue_.emplace_back(std::move(pending));
    ++stats_.accepted_scans;
    stats_.peak_scan_queue_size = std::max(stats_.peak_scan_queue_size, scan_queue_.size());
    stats_.peak_point_cloud_bytes = std::max(stats_.peak_point_cloud_bytes, queued_point_cloud_bytes_);
    condition_.notify_one();
    return InputResult::Accepted;
}

InputResult SensorDispatcher::AddWheelOdometry(OdomPtr data) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_ || stop_requested_) return InputResult::NotRunning;
  if (!options_.wheel_fusion_enabled || !data ||
      !std::isfinite(data->timestamp) || data->timestamp <= 0 ||
      !data->linear_velocity.allFinite() || !data->angular_velocity.allFinite() ||
      options_.max_wheel_queue_size == 0) {
    ++stats_.invalid_wheel;
    return InputResult::InvalidData;
  }
  const double time = data->timestamp;
  if (time < last_accepted_wheel_time_) {
    ++stats_.time_rejected_wheel;
    return InputResult::TimeDiscontinuity;
  }
  if (time == last_accepted_wheel_time_) {
    ++stats_.duplicate_wheel;
    return InputResult::TimeDiscontinuity;
  }
  if (wheel_queue_.size() >= options_.max_wheel_queue_size) {
    wheel_queue_.pop_front();
    ++stats_.dropped_wheel;
  }
  wheel_queue_.push_back(std::move(data));
  last_accepted_wheel_time_ = time;
  ++stats_.accepted_wheel;
  condition_.notify_one();
  return InputResult::Accepted;
}

void SensorDispatcher::Stop(StopMode mode) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!started_) {
            return;
        }
        stop_requested_ = true;
        drain_on_stop_ = mode == StopMode::kDrain;
        if (!drain_on_stop_) {
            CancelQueuedWorkLocked();
        }
    }
    condition_.notify_one();

    if (worker_.joinable()) {
        worker_.join();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    started_ = false;
}

bool SensorDispatcher::IsRunning() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return started_ && !stop_requested_;
}

bool SensorDispatcher::IsFaulted() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return faulted_;
}

SensorDispatcher::Stats SensorDispatcher::GetStats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

bool SensorDispatcher::IsValidImu(const IMUPtr& imu) {
    if (!imu || !std::isfinite(imu->timestamp) || imu->timestamp < 0.0) {
        return false;
    }
    return imu->angular_velocity.allFinite() && imu->linear_acceleration.allFinite();
}

bool SensorDispatcher::IsValidScan(const CloudPtr& scan) {
  if (!scan || scan->header.stamp == 0 || scan->empty()) return false;
  for (const auto& point : *scan) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.intensity) ||
        !std::isfinite(point.time)) return false;
  }
  return true;
}

bool SensorDispatcher::SelectTaskLocked(Task& task) {
    task = Task();

    if (!wheel_queue_.empty()) {
        task.type = TaskType::kWheel;
        task.wheel = std::move(wheel_queue_.front());
        wheel_queue_.pop_front();
        return true;
    }

    if (!imu_queue_.empty() &&
        (scan_queue_.empty() || imu_queue_.front()->timestamp <=
                                    math::ToSec(scan_queue_.front().data->header.stamp) ||
         last_processed_imu_time_ < math::ToSec(scan_queue_.front().data->header.stamp))) {
        task.type = TaskType::kImu;
        task.imu = std::move(imu_queue_.front());
        imu_queue_.pop_front();
        last_processed_imu_time_ = task.imu->timestamp;
        return true;
    }

    if (!scan_queue_.empty() && last_processed_imu_time_ >=
                                    math::ToSec(scan_queue_.front().data->header.stamp)) {
        const PendingScan& scan = scan_queue_.front();
        task.type = TaskType::kScan;
        task.scan = scan.data;
        queued_point_cloud_bytes_ -= scan.bytes;
        scan_queue_.pop_front();
        return true;
    }

    return false;
}

void SensorDispatcher::CancelQueuedWorkLocked() {
    stats_.cancelled_wheel += wheel_queue_.size();
    wheel_queue_.clear();
    stats_.cancelled_imu += imu_queue_.size();
    stats_.cancelled_scans += scan_queue_.size();
    imu_queue_.clear();
    scan_queue_.clear();
    queued_point_cloud_bytes_ = 0;
}

void SensorDispatcher::Run() {
    while (true) {
        Task task;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this]() {
              return stop_requested_ || !imu_queue_.empty() ||
                     !wheel_queue_.empty() ||
                     (!scan_queue_.empty() && last_processed_imu_time_ >=
                                                 math::ToSec(scan_queue_.front().data->header.stamp));
            });

            if (stop_requested_ && !drain_on_stop_) {
                break;
            }

            if (!SelectTaskLocked(task)) {
                if (stop_requested_) {
                    CancelQueuedWorkLocked();
                    break;
                }
                continue;
            }
        }

        try {
            if (task.type == TaskType::kImu) {
                if (imu_callback_) {
                    imu_callback_(task.imu);
                }
                std::lock_guard<std::mutex> lock(mutex_);
                ++stats_.processed_imu;
            } else if (task.type == TaskType::kWheel) {
              if (wheel_callback_) wheel_callback_(task.wheel);
              std::lock_guard<std::mutex> lock(mutex_);
              ++stats_.processed_wheel;
            } else if (task.type == TaskType::kScan) {
              if (scan_callback_) {
                scan_callback_(task.scan);
              }
              std::lock_guard<std::mutex> lock(mutex_);
              ++stats_.processed_scans;
            }
        } catch (const std::exception& exception) {
            LOG(ERROR) << "sensor dispatcher callback failed: " << exception.what();
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.callback_failures;
            faulted_ = true;
            stop_requested_ = true;
            drain_on_stop_ = false;
            CancelQueuedWorkLocked();
        } catch (...) {
            LOG(ERROR) << "sensor dispatcher callback failed with an unknown exception";
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.callback_failures;
            faulted_ = true;
            stop_requested_ = true;
            drain_on_stop_ = false;
            CancelQueuedWorkLocked();
        }
    }
}

}  // namespace lightning::sys
