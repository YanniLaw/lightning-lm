#pragma once

#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "common/grid_map_data.h"
#include "core/localization/localization_result.h"

namespace lightning::ros {

builtin_interfaces::msg::Time ToRosTime(double seconds);

geometry_msgs::msg::TransformStamped ToTransform(const loc::LocalizationResult& result,
                                                 const std::string& frame_id = "map",
                                                 const std::string& child_frame_id = "base_link");

nav_msgs::msg::OccupancyGrid ToOccupancyGrid(const GridMapData& map,
                                             const std::string& frame_id = "map");

}  // namespace lightning::ros
