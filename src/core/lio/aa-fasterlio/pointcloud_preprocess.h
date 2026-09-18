#ifndef FASTER_LIO_POINTCLOUD_PROCESSING_H
#define FASTER_LIO_POINTCLOUD_PROCESSING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/point_def.h"
#include "common/sensor_data.h"

namespace lightning {

/**
 * point cloud preprocess
 * just unify the point format from livox/velodyne to PCL
 *
 * 预处理程序
 * 主要是对各种不同的雷达处理时间戳差异
 */
class PointCloudPreprocess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudPreprocess() = default;
    ~PointCloudPreprocess() = default;

    /// Convert a transport-independent scan into the point representation used by LIO.
    bool Process(const TimedPointCloudData& scan, PointCloudType::Ptr& pcl_out) const;

    void Set(LidarType lid_type, double bld, int pfilt_num);

    // accessors
    double &Blind() { return blind_; }
    int &NumScans() { return num_scans_; }
    int &PointFilterNum() { return point_filter_num_; }
    float& TimeScale() { return time_scale_; }
    LidarType GetLidarType() const { return lidar_type_; }
    void SetLidarType(LidarType lt) { lidar_type_ = lt; }

    void SetHeightROI(float height_max, float height_min) {
        height_max_ = height_max;
        height_min_ = height_min;
    }

   private:
    bool ProcessLivox(const TimedPointCloudData& scan, PointCloudType& pcl_out) const;
    bool ProcessStandard(const TimedPointCloudData& scan, PointCloudType& pcl_out) const;
    bool IsInHeightRoi(const RawLidarPoint& point) const;
    static PointType ToPoint(const RawLidarPoint& point, double time_ms);

    LidarType lidar_type_ = LidarType::AVIA;
    int point_filter_num_ = 1;
    int num_scans_ = 6;
    double blind_ = 0.01;
    float time_scale_ = 1e-3;
    bool given_offset_time_ = false;

    float height_max_ = 1.0;
    float height_min_ = -1.0;
};
}  // namespace lightning

#endif
