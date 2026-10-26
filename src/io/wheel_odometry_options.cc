#include "io/wheel_odometry_options.h"

#include <cmath>
#include <utility>

namespace lightning {
namespace {

bool ReadExtrinsic(const YAML::Node& node, WheelOdometryOptions* options,
                   std::string* error) {
  const auto translation = node["extrinsic_T"];
  const auto rotation = node["extrinsic_R"];
  if (!translation && !rotation) {
    if (!options->fusion_enabled) return true;
    *error =
        "wheel_odometry.extrinsic_T/R are required when fusion is enabled";
    return false;
  }
  if (!translation || !rotation || !translation.IsSequence() ||
      !rotation.IsSequence() || translation.size() != 3 ||
      rotation.size() != 9) {
    *error =
        "wheel_odometry.extrinsic_T/R must both be configured (3/9 elements)";
    return false;
  }
  Vec3d t;
  Mat3d r;
  for (int row = 0; row < 3; ++row) {
    t[row] = translation[row].as<double>();
    for (int col = 0; col < 3; ++col)
      r(row, col) = rotation[row * 3 + col].as<double>();
  }
  if (!t.allFinite() || !r.allFinite() ||
      (r.transpose() * r - Mat3d::Identity()).norm() > 1e-6 ||
      std::abs(r.determinant() - 1.0) > 1e-6) {
    *error =
        "wheel_odometry.extrinsic_T/R must describe a finite rigid L -> B "
        "transform";
    return false;
  }
  Eigen::Quaterniond quaternion(r);
  const SE3 wheel_from_lidar(quaternion.normalized(), t);
  options->lidar_from_wheel = wheel_from_lidar.inverse();
  return true;
}

}  // namespace

bool ParseWheelOdometryOptions(const YAML::Node& root,
                               WheelOdometryOptions& output,
                               std::string& error) {
  WheelOdometryOptions parsed;
  error.clear();
  try {
    const auto system = root["system"];
    if (system && system["with_wheel_odometry_fusion"]) {
      parsed.fusion_enabled = system["with_wheel_odometry_fusion"].as<bool>();
    }
    const auto node = root["wheel_odometry"];
    if (!node) {
      if (parsed.fusion_enabled) {
        error = "wheel_odometry must be configured when "
                "system.with_wheel_odometry_fusion is true";
        return false;
      }
      output = parsed;
      return true;
    }
    if (!node.IsMap()) {
      error = "wheel_odometry must be a mapping";
      return false;
    }
    for (const auto& [key, field] :
         {std::pair{"default_velocity_variance",
                    &parsed.default_velocity_variance},
          {"nhc_lateral_variance", &parsed.nhc_lateral_variance},
          {"nhc_vertical_variance", &parsed.nhc_vertical_variance}}) {
      if (node[key]) *field = node[key].as<double>();
      if (!std::isfinite(*field) || *field <= 0.0) {
        error = std::string("Invalid wheel_odometry.") + key;
        return false;
      }
    }
    if (node["wheel_time_offset"])
      parsed.wheel_time_offset = node["wheel_time_offset"].as<double>();
    if (!std::isfinite(parsed.wheel_time_offset)) {
      error = "Invalid wheel_odometry.wheel_time_offset";
      return false;
    }
    if (!ReadExtrinsic(node, &parsed, &error)) return false;
  } catch (const YAML::Exception& exception) {
    error = std::string("Invalid wheel_odometry configuration: ") +
            exception.what();
    return false;
  }
  output = std::move(parsed);
  return true;
}

}  // namespace lightning
