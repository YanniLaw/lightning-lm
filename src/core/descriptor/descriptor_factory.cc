// Copyright 2026

#include "core/descriptor/descriptor_factory.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <type_traits>

#include "core/descriptor/std_descriptor.h"

namespace lightning {

std::unique_ptr<LoopDescriptor> CreateLoopDescriptor(
    const std::string& type, const YAML::Node& descriptor_config) {
  if (type == "none") {
    return nullptr;
  }
  if (type != "std") {
    LOG(ERROR) << "Unsupported loop descriptor: " << type;
    return nullptr;
  }

  StdDescriptor::Options options;
  try {
    const YAML::Node node =
        descriptor_config ? descriptor_config["std"] : YAML::Node();
    if (node) {
      const auto read = [&node](const char* name, auto& value) {
        if (node[name]) {
          value = node[name].as<std::decay_t<decltype(value)>>();
        }
      };
      read("maximum_corner_num", options.maximum_corner_num);
      read("plane_detection_thre", options.plane_detection_thre);
      read("plane_merge_normal_thre", options.plane_merge_normal_thre);
      read("voxel_size", options.voxel_size);
      read("voxel_init_num", options.voxel_init_num);
      read("proj_image_resolution", options.proj_image_resolution);
      read("proj_dis_min", options.proj_dis_min);
      read("proj_dis_max", options.proj_dis_max);
      read("corner_thre", options.corner_thre);
      read("descriptor_near_num", options.descriptor_near_num);
      read("descriptor_min_len", options.descriptor_min_len);
      read("descriptor_max_len", options.descriptor_max_len);
      read("non_max_suppression_radius", options.non_max_suppression_radius);
      read("std_side_resolution", options.std_side_resolution);
      read("candidate_num", options.candidate_num);
      read("vertex_diff_threshold", options.vertex_diff_threshold);
      read("rough_dis_threshold", options.rough_dis_threshold);
      read("normal_threshold", options.normal_threshold);
      read("dis_threshold", options.dis_threshold);
      read("icp_threshold", options.icp_threshold);
    }
  } catch (const YAML::Exception& exception) {
    LOG(ERROR) << "Invalid STD configuration: " << exception.what();
    return nullptr;
  }
  const auto all_positive = [](std::initializer_list<double> values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value) && value > 0.0;
    });
  };
  const auto all_nonnegative = [](std::initializer_list<double> values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value) && value >= 0.0;
    });
  };
  if (!all_positive({options.voxel_size, options.proj_image_resolution,
                     options.corner_thre, options.descriptor_min_len,
                     options.descriptor_max_len, options.std_side_resolution}) ||
      !all_nonnegative({options.plane_detection_thre,
                        options.plane_merge_normal_thre, options.proj_dis_min,
                        options.proj_dis_max, options.non_max_suppression_radius,
                        options.vertex_diff_threshold, options.rough_dis_threshold,
                        options.normal_threshold, options.dis_threshold,
                        options.icp_threshold}) ||
      options.maximum_corner_num < 3 || options.voxel_init_num < 3 ||
      options.descriptor_near_num < 3 || options.candidate_num < 1 ||
      options.descriptor_max_len <= options.descriptor_min_len ||
      options.proj_dis_max <= options.proj_dis_min || options.icp_threshold > 1.0) {
    LOG(ERROR) << "Invalid STD settings";
    return nullptr;
  }
  return std::make_unique<StdDescriptor>(options);
}

}  // namespace lightning
