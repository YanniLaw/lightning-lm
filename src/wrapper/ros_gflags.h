#ifndef LIGHTNING_ROS_GFLAGS_H
#define LIGHTNING_ROS_GFLAGS_H

#include <gflags/gflags.h>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace lightning {

/// Initialize ROS with the original arguments, then parse only non-ROS arguments with gflags.
inline void InitROSAndParseGFlags(int argc, char** argv) {
    std::vector<std::string> non_ros_arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);
    std::vector<char*> gflags_arguments;
    gflags_arguments.reserve(non_ros_arguments.size());
    for (std::string& argument : non_ros_arguments) {
        gflags_arguments.push_back(argument.data());
    }

    int gflags_argc = static_cast<int>(gflags_arguments.size());
    char** gflags_argv = gflags_arguments.data();
    google::ParseCommandLineFlags(&gflags_argc, &gflags_argv, true);
}

}  // namespace lightning

#endif  // LIGHTNING_ROS_GFLAGS_H
