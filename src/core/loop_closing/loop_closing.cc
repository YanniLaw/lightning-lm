// Copyright 2026
//
// Loop candidate detection and constraint construction.

#include "core/loop_closing/loop_closing.h"

#include "core/loop_closing/keyframe_collection.h"
#include "utils/pointcloud_utils.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>

#include <algorithm>
#include <cmath>
#include <utility>

#include <glog/logging.h>

namespace lightning {

LoopClosing::LoopClosing(Options options) : options_(std::move(options)) {}

void LoopClosing::SetOptions(Options options) { options_ = std::move(options); }

std::vector<LoopCandidate> LoopClosing::ComputeConstraints(const Keyframe::Ptr& current,
                                                           const KeyframeCollection& keyframes) {
    if (!current) {
        return {};
    }

    auto candidates = DetectLoopCandidates(current, keyframes);
    if (options_.verbose_) {
        LOG(INFO) << "lc: get kf " << current->GetID() << " candi: " << candidates.size();
    }

    ComputeLoopCandidates(keyframes, candidates);
    return candidates;
}

std::vector<LoopCandidate> LoopClosing::DetectLoopCandidates(const Keyframe::Ptr& current,
                                                             const KeyframeCollection& keyframes) {
    std::vector<LoopCandidate> candidates;
    const auto& keyframe_list = keyframes.GetAll();
    Keyframe::Ptr check_first = nullptr;

    if (last_loop_kf_ == nullptr) {
        last_loop_kf_ = current;
        return candidates;
    }

    if (last_loop_kf_ && (current->GetID() - last_loop_kf_->GetID()) <=
                             static_cast<unsigned long>(options_.loop_kf_gap_)) {
        LOG(INFO) << "skip because last loop kf: " << last_loop_kf_->GetID();
        return candidates;
    }

    for (const auto& keyframe : keyframe_list) {
        if (check_first != nullptr &&
            std::abs(static_cast<int>(keyframe->GetID() - check_first->GetID())) <= options_.min_id_interval_) {
            // Skip a local ID interval on the same trajectory.
            continue;
        }

        if (std::abs(static_cast<int>(keyframe->GetID() - current->GetID())) < options_.closest_id_th_) {
            // Nearby keyframes on the same trajectory are not loop candidates.
            break;
        }

        const Vec3d delta = keyframe->GetOptPose().translation() - current->GetOptPose().translation();
        const double distance_2d = delta.head<2>().norm();
        if (distance_2d < options_.max_range_) {
            LoopCandidate candidate(keyframe->GetID(), current->GetID());
            candidate.Tij_ = keyframe->GetLIOPose().inverse() * current->GetLIOPose();
            candidates.emplace_back(candidate);
            check_first = keyframe;
        }
    }

    if (!candidates.empty()) {
        last_loop_kf_ = current;
    }

    if (options_.verbose_ && !candidates.empty()) {
        LOG(INFO) << "lc candi: " << candidates.size();
    }

    return candidates;
}

void LoopClosing::ComputeLoopCandidates(const KeyframeCollection& keyframes,
                                        std::vector<LoopCandidate>& candidates) {
    if (candidates.empty()) {
        return;
    }
    // 执行计算
    std::for_each(candidates.begin(), candidates.end(), [&](LoopCandidate& candidate) {
        ComputeForCandidate(keyframes, candidate);
    });
    // 保存成功的候选
    std::vector<LoopCandidate> successful_candidates;
    for (const auto& candidate : candidates) {
        if (candidate.ndt_score_ > options_.ndt_score_th_) {
            successful_candidates.emplace_back(candidate);
        }
    }

    if (options_.verbose_) {
        LOG(INFO) << "success: " << successful_candidates.size() << "/" << candidates.size();
    }

    candidates.swap(successful_candidates);
}

void LoopClosing::ComputeForCandidate(const KeyframeCollection& keyframes, LoopCandidate& candidate) {
    const int submap_index_range = 40;
    const auto keyframe1 = keyframes.Find(candidate.idx1_);
    const auto keyframe2 = keyframes.Find(candidate.idx2_);
    if (keyframe1 == nullptr || keyframe2 == nullptr) {
        LOG(WARNING) << "skip loop candidate with missing keyframe: " << candidate.idx1_ << ", "
                     << candidate.idx2_;
        candidate.ndt_score_ = 0;
        return;
    }

    const auto& keyframe_list = keyframes.GetAll();
    auto build_submap = [&](unsigned long given_id, bool build_in_world) -> CloudPtr {
        CloudPtr submap(new PointCloudType);
        const std::size_t given_index = keyframes.FindIndex(given_id);
        if (given_index == keyframe_list.size()) {
            return submap;
        }

        for (int index_offset = -submap_index_range; index_offset < submap_index_range; index_offset += 4) {
            const int keyframe_index = static_cast<int>(given_index) + index_offset;
            if (keyframe_index < 0 || keyframe_index >= static_cast<int>(keyframe_list.size())) {
                continue;
            }

            const auto& keyframe = keyframe_list[keyframe_index];
            const CloudPtr cloud = keyframe->GetCloud();
            if (!cloud || cloud->empty()) {
                continue;
            }

            SE3 world_pose = keyframe->GetOptPose();
            if (!build_in_world) {
                world_pose = keyframe_list[given_index]->GetOptPose().inverse() * world_pose;
            }

            CloudPtr transformed_cloud(new PointCloudType);
            pcl::transformPointCloud(*cloud, *transformed_cloud, world_pose.matrix());
            *submap += *transformed_cloud;
        }
        return submap;
    };

    const CloudPtr submap_keyframe1 = build_submap(keyframe1->GetID(), true);
    const CloudPtr submap_keyframe2 = keyframe2->GetCloud();
    if (!submap_keyframe2 || submap_keyframe1->empty() || submap_keyframe2->empty()) {
        candidate.ndt_score_ = 0;
        return;
    }

    Mat4f target_pose = keyframe2->GetOptPose().matrix().cast<float>();
    /// 不同分辨率下的匹配
    CloudPtr output(new PointCloudType);
    const std::vector<double> resolutions{10.0, 5.0, 2.0, 1.0};

    CloudPtr rough_map1;
    CloudPtr rough_map2;
    for (const double resolution : resolutions) {
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(0.05);
        ndt.setStepSize(0.7);
        ndt.setMaximumIterations(40);
        ndt.setResolution(resolution);

        rough_map1 = VoxelGrid(submap_keyframe1, resolution * 0.1);
        rough_map2 = VoxelGrid(submap_keyframe2, resolution * 0.1);
        ndt.setInputTarget(rough_map1);
        ndt.setInputSource(rough_map2);

        ndt.align(*output, target_pose);
        target_pose = ndt.getFinalTransformation();
        candidate.ndt_score_ = ndt.getTransformationProbability();
    }

    const Mat4d transformation = target_pose.cast<double>();
    Quatd quaternion(transformation.block<3, 3>(0, 0));
    quaternion.normalize();
    const Vec3d translation = transformation.block<3, 1>(0, 3);
    candidate.Tij_ = keyframe1->GetOptPose().inverse() * SE3(quaternion, translation);
}

}  // namespace lightning
