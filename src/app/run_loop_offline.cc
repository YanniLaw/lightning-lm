//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/lio/lio_factory.h"
#include "core/lio/lio_result_conversion.h"
#include "core/loop_closing/pose_graph.h"
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

    const YAML::Node config = YAML::LoadFile(FLAGS_config);
    std::string lio_type = "aa_fasterlio";
    if (config["system"] && config["system"]["lio_type"]) {
        lio_type = config["system"]["lio_type"].as<std::string>();
    }

    auto lio = CreateLIO(lio_type);
    if (!lio) {
        LOG(ERROR) << "failed to create lio frontend";
        return -1;
    }

    LIOOptions lio_options;
    lio_options.usage = LIOUsage::kMapping;
    if (!lio->Init(FLAGS_config, lio_options)) {
        LOG(ERROR) << "failed to init lio frontend";
        return -1;
    }

    auto ui = std::make_shared<ui::PangolinWindow>();
    ui->Init();

    PoseGraph::Options pose_graph_options;
    pose_graph_options.online_mode_ = false;
    auto pose_graph = std::make_shared<PoseGraph>(pose_graph_options);
    pose_graph->Init(FLAGS_config);

    lio->SetPredictionCallback([ui](const LIOState& state) {
        NavState nav_state;
        if (ConvertToNavState(state, nav_state)) {
            ui->UpdateNavState(nav_state);
        }
    });
    lio->SetResultCallback([&pose_graph, ui](const LIOResult& result) {
        if (result.display_cloud) {
            auto cloud = std::make_shared<PointCloudType>(*result.display_cloud);
            ui->UpdateScan(cloud, result.state.pose);
        }
        if (result.update_type == LIOUpdateType::kScanMatched && result.keyframe_selected) {
            pose_graph->AddKeyframe(result);
        }
    });

    YAML_IO yaml(FLAGS_config);
    const auto lidar_type = static_cast<LidarType>(yaml.GetValue<int>("fasterlio", "lidar_type"));
    const double velodyne_time_scale = yaml.GetValue<double>("fasterlio", "time_scale");

    rosbag
        .AddImuHandle("imu_raw",
                      [&lio](IMUPtr imu) {
                          lio->AddImu(imu);
                          return true;
                      })
        .AddPointCloud2Handle("points_raw",
                              [&lio, lidar_type, velodyne_time_scale](
                                  sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                                  TimedPointCloudData scan;
                                  if (!ros::ToTimedPointCloudData(*cloud, lidar_type, velodyne_time_scale, scan)) {
                                      return true;
                                  }
                                  lio->AddPointCloud(scan);

                                  return true;
                              })
        .Go();

    pose_graph->Stop();
    lio->Stop();
    Timer::PrintAll();

    ui->Quit();

    LOG(INFO) << "done";

    return 0;
}
