//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <memory>

#include "core/system/slam.h"
#include "ros/sensor_bridge.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include "io/yaml_io.h"

DEFINE_string(input_bag, "", "输入数据包");
DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_input_bag.empty()) {
        LOG(ERROR) << "未指定输入数据";
        return -1;
    }

    using namespace lightning;

    RosbagIO rosbag(FLAGS_input_bag);

    SlamSystem::Options options;
    options.online_mode_ = false;

    SlamSystem slam(options);

    /// 实时模式好像掉帧掉的比较厉害？

    if (!slam.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init slam";
        return -1;
    }

    lightning::YAML_IO yaml(FLAGS_config);
    const bool with_ui = yaml.GetValue<bool>("system", "with_ui");
    std::shared_ptr<ui::PangolinWindow> ui;
    if (with_ui) {
        ui = std::make_shared<ui::PangolinWindow>();
        ui->Init();
        slam.SetNavStateCallback([ui](const NavState& state) { ui->UpdateNavState(state); });
        slam.SetScanCallback([ui](const CloudPtr& cloud, const SE3& pose) { ui->UpdateScan(cloud, pose); });
        slam.SetKeyframeCallback([ui](const Keyframe::Ptr& keyframe) { ui->UpdateKF(keyframe); });
    }

    slam.StartSLAM();

    std::string lidar_topic = yaml.GetValue<std::string>("common", "lidar_topic");
    std::string livox_topic = yaml.GetValue<std::string>("common", "livox_lidar_topic");
    std::string imu_topic = yaml.GetValue<std::string>("common", "imu_topic");
    const auto lidar_type = static_cast<lightning::LidarType>(yaml.GetValue<int>("fasterlio", "lidar_type"));
    const double velodyne_time_scale = yaml.GetValue<double>("fasterlio", "time_scale");

    rosbag
        /// IMU 的处理
        .AddImuHandle(imu_topic,
                      [&slam](IMUPtr imu) {
                          slam.ProcessIMU(imu);
                          return true;
                      })

        /// lidar 的处理
        .AddPointCloud2Handle(lidar_topic,
                              [&slam, lidar_type, velodyne_time_scale](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                                  TimedPointCloudData scan;
                                  if (lightning::ros::ToTimedPointCloudData(*msg, lidar_type, velodyne_time_scale,
                                                                             scan)) {
                                      slam.ProcessLidar(scan);
                                  }
                                  return true;
                              })
        /// livox 的处理
        .AddLivoxCloudHandle(livox_topic,
                             [&slam](livox_ros_driver2::msg::CustomMsg::SharedPtr cloud) {
                                 TimedPointCloudData scan;
                                 if (lightning::ros::ToTimedPointCloudData(*cloud, scan)) {
                                     slam.ProcessLidar(scan);
                                 }
                                 return true;
                             })
        .Go();

    slam.SaveMap("new_map");
    Timer::PrintAll();

    if (ui) {
        ui->Quit();
    }

    LOG(INFO) << "done";

    return 0;
}
