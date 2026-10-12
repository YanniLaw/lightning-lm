#include "core/maps/keyframe_map.h"

#include <glog/logging.h>

#include <utility>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace lightning {

CloudPtr BuildKeyframeMap(const std::vector<KeyframeMapEntry>& keyframes,
                          bool use_lio_pose,
                          bool use_voxel,
                          float resolution) {
    auto global_map = std::make_shared<PointCloudType>();
    if (keyframes.empty()) {
        LOG(WARNING) << "No keyframes provided, returning empty global map.";
        return global_map;
    }

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(resolution, resolution, resolution);

    for (const auto& keyframe : keyframes) {
        if (!keyframe.cloud) {
            continue;
        }

        auto cloud_filter = std::make_shared<PointCloudType>();
        if (use_voxel) {
            voxel.setInputCloud(keyframe.cloud);
            voxel.filter(*cloud_filter);
        } else {
            cloud_filter = std::make_shared<PointCloudType>(*keyframe.cloud);
        }

        auto cloud_transformed = std::make_shared<PointCloudType>();
        const SE3 pose = use_lio_pose ? keyframe.lio_pose : keyframe.optimized_pose;
        pcl::transformPointCloud(*cloud_filter, *cloud_transformed, pose.matrix());
        *global_map += *cloud_transformed;
    }

    if (use_voxel && !global_map->empty()) {
        auto filtered_map = std::make_shared<PointCloudType>();
        voxel.setInputCloud(global_map);
        voxel.filter(*filtered_map);
        global_map = std::move(filtered_map);
    }

    global_map->height = 1;
    global_map->width = static_cast<std::uint32_t>(global_map->size());
    global_map->is_dense = false;
    
    LOG(INFO) << "Built keyframe map with " << global_map->size() << " points.";
    return global_map;
}

}  // namespace lightning
