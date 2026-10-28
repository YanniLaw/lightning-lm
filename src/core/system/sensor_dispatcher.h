#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "common/imu.h"
#include "common/odom.h"
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
      bool wheel_fusion_enabled = false;
      std::size_t max_wheel_queue_size = 512;
      std::size_t max_imu_queue_size = 4000;
      std::size_t max_scan_queue_size = 50;
      std::size_t max_point_cloud_bytes = 128U * 1024U * 1024U;
    };

    struct Stats {
      std::size_t accepted_wheel = 0;
      std::size_t processed_wheel = 0;
      std::size_t cancelled_wheel = 0;
      std::size_t dropped_wheel = 0;
      std::size_t invalid_wheel = 0;
      std::size_t duplicate_wheel = 0;
      std::size_t time_rejected_wheel = 0;
      std::size_t accepted_imu = 0;
      std::size_t accepted_scans = 0;
      std::size_t processed_imu = 0;
      std::size_t processed_scans = 0;
      std::size_t rejected_invalid = 0;
      std::size_t rejected_time = 0;
      std::size_t rejected_queue_full = 0;
      std::size_t cancelled_imu = 0;
      std::size_t cancelled_scans = 0;
      std::size_t callback_failures = 0;
      std::size_t peak_imu_queue_size = 0;
      std::size_t peak_scan_queue_size = 0;
      std::size_t peak_point_cloud_bytes = 0;
    };

    using ImuCallback = std::function<void(const IMUPtr&)>;
    using ScanCallback = std::function<void(const CloudPtr&)>;
    using WheelCallback = std::function<void(const OdomPtr&)>;

    SensorDispatcher(Options options, ImuCallback imu_callback,
                     ScanCallback scan_callback,
                     WheelCallback wheel_callback = {});
    ~SensorDispatcher();

    SensorDispatcher(const SensorDispatcher&) = delete;
    SensorDispatcher& operator=(const SensorDispatcher&) = delete;

    /// Start the worker.  Calling Start more than once has no effect.
    void Start();

    /// Add an IMU sample without waiting for the worker.
    InputResult AddImu(IMUPtr imu);

    /// Add a shared scan without copying its point array.
    InputResult AddPointCloud(CloudPtr scan);
    InputResult AddWheelOdometry(OdomPtr data);

    /// Stop accepting input and wait for the worker according to mode.
    /// Drain drops scans without a starting IMU; LIO waits for scan-tail IMU.
    /// Cancel drops all queued work immediately.
    void Stop(StopMode mode = StopMode::kDrain);

    bool IsRunning() const;
    bool IsFaulted() const;
    Stats GetStats() const;

   private:
    struct PendingScan {
        CloudPtr data;
        std::size_t bytes = 0;
    };

    enum class TaskType {
      kNone,
      kImu,
      kScan,
      kWheel,
    };

    struct Task {
        TaskType type = TaskType::kNone;
        IMUPtr imu;
        CloudPtr scan;
        OdomPtr wheel;
    };

    static bool IsValidImu(const IMUPtr& imu);
    static bool IsValidScan(const CloudPtr& scan);

    bool SelectTaskLocked(Task& task);
    void CancelQueuedWorkLocked();
    void Run();

    const Options options_;
    const ImuCallback imu_callback_;
    const ScanCallback scan_callback_;
    const WheelCallback wheel_callback_;

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<IMUPtr> imu_queue_;
    std::deque<PendingScan> scan_queue_;
    std::deque<OdomPtr> wheel_queue_;
    std::size_t queued_point_cloud_bytes_ = 0;

    bool started_ = false;
    bool stop_requested_ = false;
    bool drain_on_stop_ = true;
    bool faulted_ = false;
    std::thread worker_;

    double last_accepted_imu_time_ = -1.0;
    double last_accepted_scan_time_ = -1.0;
    double last_processed_imu_time_ = -1.0;
    double last_accepted_wheel_time_ = -1.0;
    Stats stats_;
};

}  // namespace lightning::sys
