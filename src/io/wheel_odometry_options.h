#ifndef LIGHTNING_IO_WHEEL_ODOMETRY_OPTIONS_H_
#define LIGHTNING_IO_WHEEL_ODOMETRY_OPTIONS_H_

#include <yaml-cpp/yaml.h>

#include <string>

#include "common/wheel_odometry_options.h"

namespace lightning {

// Leaves output unchanged on failure. Enabled fusion requires a complete,
// valid L -> B extrinsic pair in YAML; stores its B -> L inverse for fusion.
bool ParseWheelOdometryOptions(const YAML::Node& root,
                               WheelOdometryOptions& output,
                               std::string& error);

}  // namespace lightning

#endif  // LIGHTNING_IO_WHEEL_ODOMETRY_OPTIONS_H_
