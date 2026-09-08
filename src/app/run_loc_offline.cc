//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <map>
#include <memory>

#include "core/localization/localization.h"
#include "ros/sensor_bridge.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include "io/yaml_io.h"

DEFINE_string(input_bag, "", "输入数据包");
DEFINE_string(config, "./config/default.yaml", "配置文件");
DEFINE_string(map_path, "./data/new_map/", "地图路径");

/// 运行定位的测试
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

    loc::Localization::Options options;
    options.online_mode_ = false;

    lightning::YAML_IO yaml(FLAGS_config);
    const bool with_ui = yaml.GetValue<bool>("system", "with_ui");
    std::shared_ptr<ui::PangolinWindow> ui;
    if (with_ui) {
        ui = std::make_shared<ui::PangolinWindow>();
        ui->SetCurrentScanSize(1);
        ui->Init();
    }

    loc::Localization loc(options);
    if (ui) {
        loc.SetNavStateCallback([ui](const NavState& state) { ui->UpdateNavState(state); });
        loc.SetRecentPoseCallback([ui](const SE3& pose) { ui->UpdateRecentPose(pose); });
        loc.SetScanCallback([ui](const CloudPtr& cloud, const SE3& pose) { ui->UpdateScan(cloud, pose); });
        loc.SetMapUpdateCallback(
            [ui](const std::map<int, CloudPtr>& static_cloud,
                 const std::map<int, CloudPtr>& dynamic_cloud) {
                ui->UpdatePointCloudGlobal(static_cloud);
                ui->UpdatePointCloudDynamic(dynamic_cloud);
            });
    }
    if (!loc.Init(FLAGS_config, FLAGS_map_path)) {
        if (ui) {
            ui->Quit();
        }
        LOG(ERROR) << "failed to init localization";
        return -1;
    }

    std::string lidar_topic = yaml.GetValue<std::string>("common", "lidar_topic");
    std::string livox_topic = yaml.GetValue<std::string>("common", "livox_lidar_topic");
    std::string imu_topic = yaml.GetValue<std::string>("common", "imu_topic");
    const auto lidar_type = static_cast<lightning::LidarType>(yaml.GetValue<int>("fasterlio", "lidar_type"));
    const double velodyne_time_scale = yaml.GetValue<double>("fasterlio", "time_scale");

    rosbag
        .AddImuHandle(imu_topic,
                      [&loc](IMUPtr imu) {
                          loc.ProcessIMU(imu);
                          usleep(1000);
                          return true;
                      })
        .AddPointCloud2Handle(lidar_topic,
                              [&loc, lidar_type, velodyne_time_scale](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                                  TimedPointCloudData scan;
                                  if (lightning::ros::ToTimedPointCloudData(*cloud, lidar_type, velodyne_time_scale,
                                                                             scan)) {
                                      loc.ProcessLidar(scan);
                                  }
                                  usleep(1000);
                                  return true;
                              })
        .AddLivoxCloudHandle(livox_topic,
                             [&loc](livox_ros_driver2::msg::CustomMsg::SharedPtr cloud) {
                                 TimedPointCloudData scan;
                                 if (lightning::ros::ToTimedPointCloudData(*cloud, scan)) {
                                     loc.ProcessLidar(scan);
                                 }
                                 usleep(1000);
                                 return true;
                             })
        .Go();

    Timer::PrintAll();
    loc.Finish();
    if (ui) {
        ui->Quit();
    }

    LOG(INFO) << "done";

    return 0;
}
