#include "core/lio/pointcloud_preprocess.h"
#include <execution>

#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <vector>

#include "wrapper/ros_utils.h"

namespace lightning {

bool PointCloudPreprocess::Init(const YAML::Node &yaml) {
    try {
        const auto node = yaml["fasterlio"];
        const int type = node["lidar_type"].as<int>();
        if (type < 1 || type > 4) {
            LOG(ERROR) << "Unsupported fasterlio.lidar_type: " << type;
            return false;
        }
        lidar_type_ = static_cast<LidarType>(type);
        blind_ = node["blind"].as<double>();
        time_scale_ = node["time_scale"].as<double>();
        num_scans_ = node["scan_line"].as<int>();
        point_filter_num_ = node["point_filter_num"].as<int>();
        height_min_ = yaml["roi"]["height_min"].as<float>();
        height_max_ = yaml["roi"]["height_max"].as<float>();
        if (!std::isfinite(blind_) || blind_ < 0.0 ||
            !std::isfinite(time_scale_) || time_scale_ < 0.0 || num_scans_ <= 0 ||
            point_filter_num_ <= 0 || !std::isfinite(height_min_) ||
            !std::isfinite(height_max_) || height_min_ > height_max_) {
            LOG(ERROR) << "Invalid LiDAR preprocessing parameters";
            return false;
        }
    } catch (const YAML::Exception &exception) {
        LOG(ERROR) << "Invalid LiDAR preprocessing configuration: " << exception.what();
        return false;
    }
    return true;
}

bool PointCloudPreprocess::Process(const sensor_msgs::msg::PointCloud2 &msg, TimedPointCloud &output) {
    switch (lidar_type_) {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        case LidarType::ROBOSENSE:
            RoboSenseHandler(msg);
            break;

        default:
            LOG(ERROR) << "Error LiDAR Type";
            return false;
    }
    output.timestamp = ToSec(msg.header.stamp);
    output.cloud = std::make_shared<PointCloudType>(cloud_out_);
    pcl_conversions::toPCL(msg.header, output.cloud->header);
    return !output.cloud->empty();
}

bool PointCloudPreprocess::Process(const livox_ros_driver2::msg::CustomMsg &msg, TimedPointCloud &output) {
    if (lidar_type_ != LidarType::AVIA || msg.point_num < 2 || msg.point_num > msg.points.size()) {
        return false;
    }
    HandleLivox(msg);
    output.timestamp = ToSec(msg.header.stamp);
    output.cloud = std::make_shared<PointCloudType>(cloud_out_);
    pcl_conversions::toPCL(msg.header, output.cloud->header);
    return !output.cloud->empty();
}

void PointCloudPreprocess::HandleLivox(const livox_ros_driver2::msg::CustomMsg &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    int plsize = msg.point_num;

    cloud_out_.reserve(plsize);
    cloud_full_.resize(plsize);

    std::vector<char> is_valid_pt(plsize, 0);
    std::vector<uint> index(plsize - 1);
    for (uint i = 0; i < plsize - 1; ++i) {
        index[i] = i + 1;  // 从1开始
    }
    // TODO: 这里的点云过滤可能会有点问题，最好考虑到线号line
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
        // if ((msg.points[i].line < num_scans_) &&
        // ((msg.points[i].tag & 0x30) == 0x10 || (msg.points[i].tag & 0x30) == 0x00)) {
        if (i % point_filter_num_ != 0) {
            return;
        }

        cloud_full_[i].x = msg.points[i].x;
        cloud_full_[i].y = msg.points[i].y;
        cloud_full_[i].z = msg.points[i].z;
        cloud_full_[i].intensity = msg.points[i].reflectivity;

        // Time of each point since scan start, in seconds.
        // 这里直接将offset_time的ns时间类型转换成秒
        cloud_full_[i].time = msg.points[i].offset_time / double(1000000000);

        if (cloud_full_[i].z < height_min_ || cloud_full_[i].z > height_max_) {
            return;
        }

        if ((abs(cloud_full_[i].x - cloud_full_[i - 1].x) > 1e-7) ||
            (abs(cloud_full_[i].y - cloud_full_[i - 1].y) > 1e-7) ||
            (abs(cloud_full_[i].z - cloud_full_[i - 1].z) > 1e-7) &&
                (cloud_full_[i].x * cloud_full_[i].x + cloud_full_[i].y * cloud_full_[i].y +
                     cloud_full_[i].z * cloud_full_[i].z >
                 (blind_ * blind_))) {
            is_valid_pt[i] = 1;
        }

        // }
    });

    for (uint i = 1; i < plsize; i++) {
        if (is_valid_pt[i]) {
            cloud_out_.points.push_back(cloud_full_[i]);
        }
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

void PointCloudPreprocess::Oust64Handler(const sensor_msgs::msg::PointCloud2 &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    for (int i = 0; i < pl_orig.points.size(); i++) {
        if (i % point_filter_num_ != 0) {
            continue;
        }

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        if (range < (blind_ * blind_)) {
            continue;
        }

        if (pl_orig.points[i].z < height_min_ || pl_orig.points[i].z > height_max_) {
            continue;
        }

        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        // 将点云的时间戳从纳秒转换为秒
        added_pt.time = pl_orig.points[i].t / 1e9;
        cloud_out_.points.push_back(added_pt);
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

void PointCloudPreprocess::RoboSenseHandler(const sensor_msgs::msg::PointCloud2 &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<PointRobotSense> pl_orig;
    pcl::fromROSMsg(msg, pl_orig);

    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    double head_time = ToSec(msg.header.stamp);

    /// RoboSense的时间戳是double，单位为秒，减去header time得到扫描内的时间偏移。

    for (int i = 0; i < pl_orig.points.size(); i++) {
        if (i % point_filter_num_ != 0) {
            continue;
        }

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        if (range < (blind_ * blind_)) {
            continue;
        }

        if (pl_orig.points[i].z < height_min_ || pl_orig.points[i].z > height_max_) {
            continue;
        }

        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;

        added_pt.time = pl_orig.points[i].timestamp - head_time; // 单位: s

        cloud_out_.points.push_back(added_pt);
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

void PointCloudPreprocess::VelodyneHandler(const sensor_msgs::msg::PointCloud2 &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) {
        return;
    }
    cloud_out_.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 3.61;  // scan angular velocity
    std::vector<bool> is_first(num_scans_, true);
    std::vector<double> yaw_fp(num_scans_, 0.0);    // yaw of first scan point
    std::vector<float> yaw_last(num_scans_, 0.0);   // yaw of last scan point
    std::vector<float> time_last(num_scans_, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0) {
        given_offset_time_ = true;
    } else {
        given_offset_time_ = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--) {
            if (pl_orig.points[i].ring == layer_first) {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    for (int i = 0; i < plsize; i++) {
        PointType added_pt;

        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.time = pl_orig.points[i].time * time_scale_;  // curvature unit: ms

        if (!given_offset_time_) {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.time = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.time;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.time = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.time = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.time < time_last[layer]) {
                added_pt.time += 360.0 / omega_l;
            }

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.time;
        }

        added_pt.time *= 1e-3;  // Convert the original millisecond calculation to seconds.

        if (i % point_filter_num_ == 0) {
            if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind_ * blind_)) {
                cloud_out_.points.push_back(added_pt);
            }
        }
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

}  // namespace lightning
