#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "common/imu.h"
#include "common/sensor_data.h"

namespace lightning::sys {

/**
 * Owns the transport-independent input queues and executes sensor callbacks
 * from one worker thread.  Producers never wait for a callback to finish.
 */
class SensorDispatcher final {
   public:
    enum class StopMode {
        kDrain,
        kCancel,
    };

    struct Options {
        std::size_t max_imu_queue_size = 4000;
        std::size_t max_scan_queue_size = 50;
        std::size_t max_point_cloud_bytes = 128U * 1024U * 1024U;
    };

    struct Stats {
        std::size_t accepted_imu = 0;
        std::size_t accepted_scans = 0;
        std::size_t processed_imu = 0;
        std::size_t processed_scans = 0;
        std::size_t rejected_invalid = 0;
        std::size_t rejected_time = 0;
        std::size_t rejected_queue_full = 0;
        std::size_t cancelled_imu = 0;
        std::size_t cancelled_scans = 0;
        std::size_t incomplete_scans = 0;
        std::size_t callback_failures = 0;
        std::size_t peak_imu_queue_size = 0;
        std::size_t peak_scan_queue_size = 0;
        std::size_t peak_point_cloud_bytes = 0;
    };

    using ImuCallback = std::function<void(const IMUPtr&)>;
    using ScanCallback = std::function<void(const TimedPointCloudData&)>;

    SensorDispatcher(Options options, ImuCallback imu_callback, ScanCallback scan_callback);
    ~SensorDispatcher();

    SensorDispatcher(const SensorDispatcher&) = delete;
    SensorDispatcher& operator=(const SensorDispatcher&) = delete;

    /// Start the worker.  Calling Start more than once has no effect.
    void Start();

    /// Add an IMU sample without waiting for the worker.
    InputResult AddImu(IMUPtr imu);

    /// Add a shared scan without copying its point array.
    InputResult AddPointCloud(std::shared_ptr<const TimedPointCloudData> scan);

    /// Stop accepting input and wait for the worker according to mode.
    /// Drain drops scans that cannot be completed because their tail IMU is
    /// absent; Cancel drops all queued work immediately.
    void Stop(StopMode mode = StopMode::kDrain);

    bool IsRunning() const;
    bool IsFaulted() const;
    Stats GetStats() const;

   private:
    struct PendingScan {
        std::shared_ptr<const TimedPointCloudData> data;
        std::int64_t end_time_ns = 0;
        std::size_t bytes = 0;
    };

    enum class TaskType {
        kNone,
        kImu,
        kScan,
    };

    struct Task {
        TaskType type = TaskType::kNone;
        IMUPtr imu;
        std::shared_ptr<const TimedPointCloudData> scan;
    };

    static bool IsValidImu(const IMUPtr& imu);
    static bool IsValidScan(const TimedPointCloudData& scan);
    static bool ToNanoseconds(double timestamp_sec, std::int64_t& timestamp_ns);
    static std::int64_t ScanEndTimeNs(const TimedPointCloudData& scan);

    bool SelectTaskLocked(Task& task);
    void CancelQueuedWorkLocked(bool mark_incomplete_scans);
    void Run();

    const Options options_;
    const ImuCallback imu_callback_;
    const ScanCallback scan_callback_;

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<IMUPtr> imu_queue_;
    std::deque<PendingScan> scan_queue_;
    std::size_t queued_point_cloud_bytes_ = 0;

    bool started_ = false;
    bool stop_requested_ = false;
    bool drain_on_stop_ = true;
    bool faulted_ = false;
    std::thread worker_;

    std::int64_t last_accepted_imu_ns_ = -1;
    std::int64_t last_accepted_scan_ns_ = -1;
    std::int64_t last_processed_imu_ns_ = -1;
    Stats stats_;
};

}  // namespace lightning::sys
