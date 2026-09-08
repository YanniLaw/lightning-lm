//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "ros/sensor_bridge.h"
#include "ui/pangolin_window.h"
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

    LaserMapping lio;
    if (!lio.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init lio";
        return -1;
    };

    auto ui = std::make_shared<ui::PangolinWindow>();
    ui->Init();
    lio.SetNavStateCallback([ui](const NavState& state) { ui->UpdateNavState(state); });
    lio.SetScanCallback([ui](const CloudPtr& cloud, const SE3& pose) { ui->UpdateScan(cloud, pose); });

    YAML_IO yaml(FLAGS_config);
    const auto lidar_type = static_cast<LidarType>(yaml.GetValue<int>("fasterlio", "lidar_type"));
    const double velodyne_time_scale = yaml.GetValue<double>("fasterlio", "time_scale");

    auto loop = std::make_shared<LoopClosing>();
    loop->Init(FLAGS_config);

    Keyframe::Ptr cur_kf = nullptr;

    rosbag
        .AddImuHandle("imu_raw",
                      [&lio](IMUPtr imu) {
                          lio.ProcessIMU(imu);
                          return true;
                      })
        .AddPointCloud2Handle("points_raw",
                              [&lio, &cur_kf, &loop, lidar_type, velodyne_time_scale](
                                  sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                                  TimedPointCloudData scan;
                                  if (!ros::ToTimedPointCloudData(*cloud, lidar_type, velodyne_time_scale, scan)) {
                                      return true;
                                  }
                                  lio.ProcessPointCloud(scan);
                                  lio.Run();

                                  auto kf = lio.GetKeyframe();
                                  if (cur_kf != kf) {
                                      cur_kf = kf;
                                      loop->AddKF(kf);
                                  }

                                  return true;
                              })
        .Go();

    lio.SaveMap();
    Timer::PrintAll();

    ui->Quit();

    LOG(INFO) << "done";

    return 0;
}
