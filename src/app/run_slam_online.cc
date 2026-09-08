//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ros/slam_node.h"
#include "utils/timer.h"
#include "wrapper/ros_gflags.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    lightning::InitROSAndParseGFlags(argc, argv);

    auto node = std::make_shared<lightning::ros::SlamNode>(FLAGS_config);
    if (!node->Init()) {
        LOG(ERROR) << "failed to init slam";
        return -1;
    }

    rclcpp::spin(node);
    node->Stop();

    lightning::Timer::PrintAll();

    rclcpp::shutdown();

    LOG(INFO) << "done";

    return 0;
}
