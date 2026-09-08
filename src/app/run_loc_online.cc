//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ros/localization_node.h"
#include "wrapper/ros_gflags.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行定位的测试
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    lightning::InitROSAndParseGFlags(argc, argv);
    auto node = std::make_shared<lightning::ros::LocalizationNode>(FLAGS_config);
    if (!node->Init()) {
        LOG(ERROR) << "failed to init loc";
        return -1;
    }

    rclcpp::spin(node);
    node->Stop();

    rclcpp::shutdown();

    return 0;
}
