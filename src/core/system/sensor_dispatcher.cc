#include "core/system/sensor_dispatcher.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <glog/logging.h>

namespace lightning::sys {
namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

bool PointCloudBytes(const TimedPointCloudData& scan, std::size_t& bytes) {
    if (scan.points.size() > std::numeric_limits<std::size_t>::max() / sizeof(RawLidarPoint)) {
        return false;
    }
    bytes = scan.points.size() * sizeof(RawLidarPoint);
    return true;
}

}  // namespace

SensorDispatcher::SensorDispatcher(Options options, ImuCallback imu_callback, ScanCallback scan_callback)
    : options_(options), imu_callback_(std::move(imu_callback)), scan_callback_(std::move(scan_callback)) {}

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

    std::int64_t timestamp_ns = 0;
    if (!ToNanoseconds(imu->timestamp, timestamp_ns)) {
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
    if (last_accepted_imu_ns_ >= 0 && timestamp_ns < last_accepted_imu_ns_) {
        ++stats_.rejected_time;
        return InputResult::TimeDiscontinuity;
    }

    last_accepted_imu_ns_ = timestamp_ns;
    imu_queue_.emplace_back(std::move(imu));
    ++stats_.accepted_imu;
    stats_.peak_imu_queue_size = std::max(stats_.peak_imu_queue_size, imu_queue_.size());
    condition_.notify_one();
    return InputResult::Accepted;
}

InputResult SensorDispatcher::AddPointCloud(std::shared_ptr<const TimedPointCloudData> scan) {
    if (!scan || !IsValidScan(*scan)) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.rejected_invalid;
        return InputResult::InvalidData;
    }

    std::size_t bytes = 0;
    if (!PointCloudBytes(*scan, bytes) || bytes > options_.max_point_cloud_bytes) {
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

    const std::int64_t timestamp_ns = scan->timestamp_ns;
    if (last_accepted_scan_ns_ >= 0 && timestamp_ns < last_accepted_scan_ns_) {
        ++stats_.rejected_time;
        return InputResult::TimeDiscontinuity;
    }

    PendingScan pending;
    pending.data = std::move(scan);
    pending.end_time_ns = ScanEndTimeNs(*pending.data);
    pending.bytes = bytes;
    last_accepted_scan_ns_ = timestamp_ns;
    queued_point_cloud_bytes_ += bytes;
    scan_queue_.emplace_back(std::move(pending));
    ++stats_.accepted_scans;
    stats_.peak_scan_queue_size = std::max(stats_.peak_scan_queue_size, scan_queue_.size());
    stats_.peak_point_cloud_bytes = std::max(stats_.peak_point_cloud_bytes, queued_point_cloud_bytes_);
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
            CancelQueuedWorkLocked(false);
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

bool SensorDispatcher::IsValidScan(const TimedPointCloudData& scan) {
    if (scan.timestamp_ns <= 0 || scan.points.empty()) {
        return false;
    }
    for (const RawLidarPoint& point : scan.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            !std::isfinite(point.intensity)) {
            return false;
        }
    }
    return true;
}

bool SensorDispatcher::ToNanoseconds(double timestamp_sec, std::int64_t& timestamp_ns) {
    constexpr double kMaxSeconds =
        static_cast<double>(std::numeric_limits<std::int64_t>::max()) / kNanosecondsPerSecond;
    if (!std::isfinite(timestamp_sec) || timestamp_sec < 0.0 || timestamp_sec > kMaxSeconds) {
        return false;
    }
    timestamp_ns = static_cast<std::int64_t>(std::llround(timestamp_sec * kNanosecondsPerSecond));
    return true;
}

std::int64_t SensorDispatcher::ScanEndTimeNs(const TimedPointCloudData& scan) {
    std::int64_t end_time_ns = scan.timestamp_ns;
    if (!scan.has_point_time) {
        return end_time_ns;
    }

    const std::int64_t max_offset =
        std::numeric_limits<std::int64_t>::max() - scan.timestamp_ns;
    for (const RawLidarPoint& point : scan.points) {
        if (point.has_time && point.time_offset_ns <= max_offset) {
            end_time_ns = std::max(end_time_ns, scan.timestamp_ns + point.time_offset_ns);
        }
    }
    return end_time_ns;
}

bool SensorDispatcher::SelectTaskLocked(Task& task) {
    task = Task();

    if (!imu_queue_.empty() && !scan_queue_.empty()) {
        const PendingScan& scan = scan_queue_.front();
        std::int64_t queued_imu_timestamp_ns = 0;
        if (!ToNanoseconds(imu_queue_.front()->timestamp, queued_imu_timestamp_ns)) {
            return false;
        }

        if (queued_imu_timestamp_ns <= scan.end_time_ns || last_processed_imu_ns_ < scan.end_time_ns) {
            task.type = TaskType::kImu;
            task.imu = std::move(imu_queue_.front());
            imu_queue_.pop_front();
            last_processed_imu_ns_ = queued_imu_timestamp_ns;
            return true;
        }

        task.type = TaskType::kScan;
        task.scan = scan.data;
        queued_point_cloud_bytes_ -= scan.bytes;
        scan_queue_.pop_front();
        return true;
    }

    if (!imu_queue_.empty()) {
        std::int64_t queued_imu_timestamp_ns = 0;
        if (!ToNanoseconds(imu_queue_.front()->timestamp, queued_imu_timestamp_ns)) {
            return false;
        }
        task.type = TaskType::kImu;
        task.imu = std::move(imu_queue_.front());
        imu_queue_.pop_front();
        last_processed_imu_ns_ = queued_imu_timestamp_ns;
        return true;
    }

    if (!scan_queue_.empty() && last_processed_imu_ns_ >= scan_queue_.front().end_time_ns) {
        const PendingScan& scan = scan_queue_.front();
        task.type = TaskType::kScan;
        task.scan = scan.data;
        queued_point_cloud_bytes_ -= scan.bytes;
        scan_queue_.pop_front();
        return true;
    }

    return false;
}

void SensorDispatcher::CancelQueuedWorkLocked(bool mark_incomplete_scans) {
    stats_.cancelled_imu += imu_queue_.size();
    stats_.cancelled_scans += scan_queue_.size();
    if (mark_incomplete_scans) {
        stats_.incomplete_scans += scan_queue_.size();
    }
    imu_queue_.clear();
    scan_queue_.clear();
    queued_point_cloud_bytes_ = 0;
}

void SensorDispatcher::Run() {
    while (true) {
        Task task;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this]() { return stop_requested_ || !imu_queue_.empty() || !scan_queue_.empty(); });

            if (stop_requested_ && !drain_on_stop_) {
                break;
            }

            if (!SelectTaskLocked(task)) {
                if (stop_requested_) {
                    CancelQueuedWorkLocked(true);
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
            } else if (task.type == TaskType::kScan) {
                if (scan_callback_) {
                    scan_callback_(*task.scan);
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
            CancelQueuedWorkLocked(false);
        } catch (...) {
            LOG(ERROR) << "sensor dispatcher callback failed with an unknown exception";
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.callback_failures;
            faulted_ = true;
            stop_requested_ = true;
            drain_on_stop_ = false;
            CancelQueuedWorkLocked(false);
        }
    }
}

}  // namespace lightning::sys
