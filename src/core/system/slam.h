//
// Created by xiang on 25-5-6.
//

#ifndef LIGHTNING_SLAM_H
#define LIGHTNING_SLAM_H

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "common/eigen_types.h"
#include "common/grid_map_data.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "common/sensor_data.h"
#include "core/lio/lio_data.h"
#include "core/system/sensor_dispatcher.h"

namespace lightning {

class LaserMapping;  //  lio 前端
class LoopClosing;   // 回环检测

namespace g2p5 {
class G2P5;
}

/**
 * SLAM 系统调用接口
 */
class SlamSystem {
   public:
    struct Options {
        Options() {}

        bool online_mode_ = true;  // 在线模式，在线模式下会起一些子线程来做异步处理

        bool with_cc_ = true;               // 是否需要带交叉验证
        bool with_gridmap_ = true;          // 是否需要2D栅格
        bool with_loop_closing_ = true;     // 是否需要回环检测
        bool with_visualization_ = true;    // 是否请求外层可视化消费者
        bool with_2dvisualization_ = true;  // 是否请求外层2D可视化消费者
        bool with_rviz_ = false;            // 是否发布ROS2 RViz可视化话题
        double rviz_local_map_publish_hz_ = 2.0;  // 累积局部地图发布频率，<=0表示每帧发布
        std::size_t rviz_local_map_max_scans_ = 200;  // 累积局部地图最多保留的扫描帧数

        bool step_on_kf_ = true;  // 是否在关键帧处暂停p
    };

    SlamSystem(Options options);
    ~SlamSystem();

    /// 初始化
    bool Init(const std::string& yaml_path);

    /// 对外部交互接口
    /// 开始建图，输入地图名称
    void StartSLAM(/*std::string map_name*/);

    /// 保存地图，默认保存至./data/地图名/ 下方
    void SaveMap(const std::string& path = "");

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// Process a transport-independent lidar scan.
    bool ProcessLidar(const TimedPointCloudData& cloud);

    /// Register consumers before starting the system.
    void SetLIODataCallback(std::function<void(const LIOData&)> callback);
    void SetNavStateCallback(std::function<void(const NavState&)> callback);
    void SetScanCallback(std::function<void(const CloudPtr&, const SE3&)> callback);
    void SetKeyframeCallback(std::function<void(const Keyframe::Ptr&)> callback);
    void SetGridMapCallback(std::function<void(GridMapDataPtr)> callback);

    sys::SensorDispatcher::Stats GetInputStats() const;

    /// Stop processing.  Repeated calls are safe.
    void Stop();

   private:
    void ProcessIMUOnWorker(const IMUPtr& imu);
    bool ProcessLidarOnWorker(const TimedPointCloudData& cloud);
    bool RunPendingLidar();

    Options options_;
    std::atomic_bool running_ = false;

    std::string map_name_;  // 地图名

    std::shared_ptr<LaserMapping> lio_ = nullptr;       // lio 前端
    std::shared_ptr<LoopClosing> lc_ = nullptr;         // 回环检测
    std::shared_ptr<g2p5::G2P5> g2p5_ = nullptr;        // 栅格地图

    Keyframe::Ptr cur_kf_ = nullptr;
    std::function<void(const LIOData&)> lio_data_callback_;
    std::function<void(const NavState&)> nav_state_callback_;
    std::function<void(const CloudPtr&, const SE3&)> scan_callback_;
    std::function<void(const Keyframe::Ptr&)> keyframe_callback_;
    std::function<void(GridMapDataPtr)> grid_map_callback_;
    std::unique_ptr<sys::SensorDispatcher> sensor_dispatcher_;
};
}  // namespace lightning

#endif  // LIGHTNING_SLAM_H
