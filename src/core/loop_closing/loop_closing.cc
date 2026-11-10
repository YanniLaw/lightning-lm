// Copyright 2026
//
// Loop candidate detection and constraint construction.

#include "core/loop_closing/loop_closing.h"

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ndt.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <optional>
#include <utility>

#include "core/pose_graph/keyframe_collection.h"
#include "utils/pointcloud_utils.h"
#include "utils/thread_pool.h"

namespace lightning {

namespace {

struct MatchGeometry {
  double overlap_ratio = 0.0;
  double mean_squared_error = 0.0;
};

MatchGeometry EvaluateGeometry(const pcl::KdTreeFLANN<PointType>& tree,
                               const PointCloudType& source, const Mat4f& pose) {
  constexpr double kOverlapSquaredDistance = 0.2 * 0.2;
  constexpr double kMaxCorrespondenceSquaredDistance = 0.5 * 0.5;
  MatchGeometry geometry;
  std::size_t correspondence_count = 0;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  for (const auto& point : source) {
    PointType query;
    query.getVector3fMap() =
        pose.block<3, 3>(0, 0) * point.getVector3fMap() + pose.block<3, 1>(0, 3);
    if (tree.nearestKSearch(query, 1, indices, distances) != 1 ||
        distances[0] > kMaxCorrespondenceSquaredDistance) {
      continue;
    }
    const double squared_distance = distances[0];
    geometry.overlap_ratio += squared_distance <= kOverlapSquaredDistance;
    geometry.mean_squared_error += squared_distance;
    ++correspondence_count;
  }
  // Coverage uses every source point; error uses only valid correspondences.
  geometry.overlap_ratio /= source.size();
  geometry.mean_squared_error =
      correspondence_count > 0
          ? geometry.mean_squared_error / correspondence_count
          : std::numeric_limits<double>::infinity();
  return geometry;
}

}  // namespace

LoopClosing::LoopClosing(Options options) : options_(std::move(options)) {}

LoopClosing::~LoopClosing() {
    std::lock_guard<std::mutex> lock(match_mutex_);
    CHECK(finish_node_task_->GetState() == Task::State::kNew);
    CHECK(when_done_task_->GetState() == Task::State::kNew);
    CHECK(match_results_.empty());
    CHECK_EQ(num_started_nodes_, num_finished_nodes_);
    CHECK(!done_callback_);
}

void LoopClosing::SetOptions(Options options) { options_ = std::move(options); }

void LoopClosing::SetDescriptor(std::unique_ptr<LoopDescriptor> descriptor) {
  descriptor_ = std::move(descriptor);
}

void LoopClosing::SetThreadPool(ThreadPoolInterface* thread_pool) {
    std::lock_guard<std::mutex> lock(match_mutex_);
    thread_pool_ = thread_pool;
}

void LoopClosing::ScheduleMatches(std::vector<LoopMatchInput> inputs) {
    std::lock_guard<std::mutex> lock(match_mutex_);
    CHECK(thread_pool_ != nullptr);
    CHECK(!done_callback_);

    for (const LoopMatchInput& input : inputs) {
        match_results_.emplace_back(std::nullopt);
        std::optional<LoopCandidate>* const result_slot = &match_results_.back();

        auto match_task = std::make_unique<Task>();
        match_task->SetWorkItem([this, input, result_slot]() {
            try {
                *result_slot = ComputeMatch(input);
            } catch (const std::exception& exception) {
                LOG(FATAL) << "loop matching task failed for " << input.candidate.idx1_ << " -> "
                           << input.candidate.idx2_ << ": " << exception.what();
            } catch (...) {
                LOG(FATAL) << "loop matching task failed for " << input.candidate.idx1_ << " -> "
                           << input.candidate.idx2_ << " with an unknown exception";
            }
        });

        finish_node_task_->AddDependency(thread_pool_->Schedule(std::move(match_task)));
    }
}

void LoopClosing::NotifyEndOfNode() {
    std::lock_guard<std::mutex> lock(match_mutex_);
    CHECK(finish_node_task_ != nullptr);

    finish_node_task_->SetWorkItem([this]() {
        std::lock_guard<std::mutex> lock(match_mutex_);
        ++num_finished_nodes_;
    });
    const std::weak_ptr<Task> finish_node_task_handle =
        thread_pool_->Schedule(std::move(finish_node_task_));
    finish_node_task_ = std::make_unique<Task>();
    when_done_task_->AddDependency(finish_node_task_handle);
    ++num_started_nodes_;
}

int LoopClosing::GetNumFinishedNodes() const {
    std::lock_guard<std::mutex> lock(match_mutex_);
    return num_finished_nodes_;
}

int LoopClosing::GetNumPendingNodes() const {
    std::lock_guard<std::mutex> lock(match_mutex_);
    return num_started_nodes_ - num_finished_nodes_;
}

void LoopClosing::WhenDone(DoneCallback callback) {
    std::lock_guard<std::mutex> lock(match_mutex_);
    CHECK(callback);
    CHECK(!done_callback_);
    CHECK(thread_pool_ != nullptr);

    try {
        auto next_when_done_task = std::make_unique<Task>();
        done_callback_ = std::move(callback);
        CHECK(when_done_task_ != nullptr);
        when_done_task_->SetWorkItem([this]() { RunWhenDoneCallback(); });
        thread_pool_->Schedule(std::move(when_done_task_));
        when_done_task_ = std::move(next_when_done_task);
    } catch (const std::exception& exception) {
        LOG(FATAL) << "failed to schedule loop-match completion: " << exception.what();
    } catch (...) {
        LOG(FATAL) << "failed to schedule loop-match completion";
    }
}

void LoopClosing::RunWhenDoneCallback() {
    DoneCallback done_callback;
    MatchResult match_result;
    {
        std::lock_guard<std::mutex> lock(match_mutex_);
        CHECK(done_callback_);
        done_callback = std::move(done_callback_);
        for (const auto& match : match_results_) {
            if (match) {
                match_result.constraints.emplace_back(*match);
            }
        }
        LOG_IF(INFO, match_results_.size() > 0) << match_results_.size() 
            << " computations resulted in " << match_result.constraints.size() 
            << " additional constraints.";
        match_results_.clear();
        done_callback_ = nullptr;
    }

    done_callback(std::move(match_result));
}

std::vector<LoopCandidate> LoopClosing::ComputeConstraints(const Keyframe::Ptr& current,
                                                           const KeyframeCollection& keyframes) {
    if (!current) {
        return {};
    }

    const auto inputs = PrepareMatches(current, keyframes);
    std::vector<LoopCandidate> successful_candidates;
    successful_candidates.reserve(inputs.size());
    for (const auto& input : inputs) {
        const auto result = ComputeMatch(input);
        if (result) {
            successful_candidates.emplace_back(*result);
        }
    }
    if (options_.verbose_) {
        LOG(INFO) << "success: " << successful_candidates.size() << "/" << inputs.size();
    }
    return successful_candidates;
}

std::vector<LoopMatchInput> LoopClosing::PrepareMatches(const Keyframe::Ptr& current,
                                                        const KeyframeCollection& keyframes) {
    if (!current) {
        return {};
    }

    // STD processes complete consecutive windows; geometry keeps its own cadence.
    const int detection_interval =
        descriptor_ ? options_.descriptor_submap_keyframes_ : options_.detection_interval_;
    const bool first_keyframe = !has_received_keyframe_;
    if (first_keyframe) {
        has_received_keyframe_ = true;
    } else {
        ++keyframes_since_detection_;
        if (keyframes_since_detection_ < detection_interval) {
            return {};
        }
        keyframes_since_detection_ = 0;
    }

    std::vector<LoopMatchInput> inputs;
    if (descriptor_) {
        const auto cloud = BuildDescriptorCloud(current, keyframes);
        if (!cloud || cloud->empty()) {
            return inputs;
        }
        const auto& history = keyframes.GetAll();
        const std::size_t query_index = keyframes.FindIndex(current->GetID());
        const std::size_t window_size = std::min(
            query_index + 1, static_cast<std::size_t>(options_.descriptor_submap_keyframes_));
        const std::uint64_t gap = options_.descriptor_skip_near_num_;
        const std::uint64_t query_id = current->GetID();
        // Exclude recent IDs and every anchor sharing a source scan with query.
        const std::uint64_t reference_id_limit = std::min<std::uint64_t>(
            query_id > gap ? query_id - gap : 0,
            history[query_index + 1 - window_size]->GetID());
        const auto matches = descriptor_->DetectAndAdd(
            cloud, current->GetLIOPose(), query_id, reference_id_limit);
        for (const auto& match : matches) {
            auto input = BuildMatchInput(keyframes, LoopCandidate(match.reference_id, query_id));
            if (input) {
                if (match.reference_from_query) {
                    input->registration_guess =
                        (input->reference_world_pose * *match.reference_from_query).matrix().cast<float>();
                }
                inputs.emplace_back(std::move(*input));
            }
        }
    } else if (!first_keyframe) {
        const auto candidates = DetectLoopCandidates(current, keyframes);
        inputs.reserve(candidates.size());
        for (const auto& candidate : candidates) {
            const auto input = BuildMatchInput(keyframes, candidate);
            if (input) {
                inputs.emplace_back(*input);
            }
        }
    }
    if (options_.verbose_ && !inputs.empty()) {
        LOG(INFO) << "lc: get kf " << current->GetID() << " candi: " << inputs.size();
    }
    return inputs;
}

std::vector<LoopCandidate> LoopClosing::DetectLoopCandidates(const Keyframe::Ptr& current,
                                                             const KeyframeCollection& keyframes) {
    std::vector<LoopCandidate> candidates;
    const auto& keyframe_list = keyframes.GetAll();
    Keyframe::Ptr check_first = nullptr;
    const auto id_difference = [](unsigned long lhs, unsigned long rhs) {
        return lhs >= rhs ? static_cast<std::uint64_t>(lhs - rhs)
                          : static_cast<std::uint64_t>(rhs - lhs);
    };

    for (const auto& keyframe : keyframe_list) {
        if (keyframe->GetID() == current->GetID()) {
            continue;
        }
        if (check_first != nullptr &&
            id_difference(keyframe->GetID(), check_first->GetID()) <=
                static_cast<std::uint64_t>(std::max(options_.min_id_interval_, 0))) {
            // Skip a local ID interval on the same trajectory.
            continue;
        }

        if (id_difference(keyframe->GetID(), current->GetID()) <
            static_cast<std::uint64_t>(std::max(options_.closest_id_th_, 0))) {
            // Nearby keyframes on the same trajectory are not loop candidates.
            break;
        }

        const Vec3d delta = keyframe->GetOptPose().translation() - current->GetOptPose().translation();
        const double distance_2d = delta.head<2>().norm();
        if (distance_2d < options_.max_range_) {
            LoopCandidate candidate(keyframe->GetID(), current->GetID());
            candidates.emplace_back(candidate);
            check_first = keyframe;
        }
    }

    return candidates;
}

CloudConstPtr LoopClosing::BuildDescriptorCloud(
    const Keyframe::Ptr& anchor, const KeyframeCollection& keyframes) const {
  const auto& history = keyframes.GetAll();
  const std::size_t anchor_index = keyframes.FindIndex(anchor->GetID());
  if (anchor_index == history.size()) {
    return nullptr;
  }
  const std::size_t window_size = std::min(
      anchor_index + 1, static_cast<std::size_t>(options_.descriptor_submap_keyframes_));
  const SE3 anchor_from_local = anchor->GetLIOPose().inverse();
  CloudPtr accumulated(new PointCloudType);
  for (std::size_t i = anchor_index + 1 - window_size; i <= anchor_index; ++i) {
    const auto cloud = history[i]->GetCloud();
    if (!cloud || cloud->empty()) {
      continue;
    }
    CloudPtr local_cloud(new PointCloudType);
    pcl::transformPointCloud(*cloud, *local_cloud, history[i]->GetLIOPose().matrix());
    const auto sampled = VoxelGrid(local_cloud, options_.descriptor_downsample_resolution_);
    PointCloudType transformed;
    pcl::transformPointCloud(*sampled, transformed, anchor_from_local.matrix());
    *accumulated += transformed;
  }
  return accumulated;
}

std::optional<LoopMatchInput> LoopClosing::BuildMatchInput(const KeyframeCollection& keyframes,
                                                            const LoopCandidate& candidate) const {
    const int submap_index_range = 40;
    const auto keyframe1 = keyframes.Find(candidate.idx1_);
    const auto keyframe2 = keyframes.Find(candidate.idx2_);
    if (keyframe1 == nullptr || keyframe2 == nullptr) {
        LOG(WARNING) << "skip loop candidate with missing keyframe: " << candidate.idx1_ << ", "
                     << candidate.idx2_;
        return std::nullopt;
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
            const auto cloud = keyframe->GetCloud();
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

    CloudPtr submap_keyframe1;
    if (descriptor_) {
        // Rebuild the same causal geometry used by descriptor extraction, then
        // place the complete reference window using its optimized anchor pose.
        const auto reference_cloud = BuildDescriptorCloud(keyframe1, keyframes);
        submap_keyframe1 = std::make_shared<PointCloudType>();
        pcl::transformPointCloud(*reference_cloud, *submap_keyframe1, keyframe1->GetOptPose().matrix());
    } else {
        submap_keyframe1 = build_submap(keyframe1->GetID(), true);
    }
    const auto submap_keyframe2 = keyframe2->GetCloud();
    if (!submap_keyframe2 || submap_keyframe1->empty() || submap_keyframe2->empty()) {
        return std::nullopt;
    }

    LoopMatchInput input;
    input.candidate = candidate;
    input.target_cloud = submap_keyframe1;
    input.source_cloud = submap_keyframe2;
    input.reference_world_pose = keyframe1->GetOptPose();
    input.initial_source_world_pose = keyframe2->GetOptPose().matrix().cast<float>();
    return input;
}

std::optional<LoopCandidate> LoopClosing::ComputeMatch(const LoopMatchInput& input) const {
    if (!input.target_cloud || !input.source_cloud || input.target_cloud->empty() ||
        input.source_cloud->empty()) {
        return std::nullopt;
    }

    LoopCandidate candidate = input.candidate;
    Mat4f target_pose = input.registration_guess.value_or(input.initial_source_world_pose);
    double ndt_score = 0.0;
    bool converged = false;
    // A complete descriptor pose needs local refinement; geometry retains coarse search.
    CloudPtr output(new PointCloudType);
    const std::vector<double> resolutions = input.registration_guess
                                              ? std::vector<double>{1.0}
                                              : std::vector<double>{10.0, 5.0, 2.0, 1.0};

    CloudPtr rough_map1;
    CloudPtr rough_map2;
    for (const double resolution : resolutions) {
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(0.05);
        ndt.setStepSize(0.7);
        ndt.setMaximumIterations(40);
        ndt.setResolution(resolution);

        rough_map1 = VoxelGrid(input.target_cloud, resolution * 0.1);
        rough_map2 = VoxelGrid(input.source_cloud, resolution * 0.1);
        ndt.setInputTarget(rough_map1);
        ndt.setInputSource(rough_map2);

        ndt.align(*output, target_pose);
        converged = ndt.hasConverged();
        target_pose = ndt.getFinalTransformation();
        ndt_score = ndt.getTransformationProbability();
        if (!target_pose.allFinite() || !std::isfinite(ndt_score)) {
            return std::nullopt;
        }
    }
    // check if the loop candidate passes the geometry checks and the NDT score threshold
    if (!converged || ndt_score <= options_.ndt_score_th_ ||
        !PassesGeometryChecks(input, target_pose)) {
        return std::nullopt;
    }

    const Mat4d transformation = target_pose.cast<double>();
    Quatd quaternion(transformation.block<3, 3>(0, 0));
    quaternion.normalize();
    const Vec3d translation = transformation.block<3, 1>(0, 3);
    candidate.Tij_ = input.reference_world_pose.inverse() * SE3(quaternion, translation);
    return candidate;
}

bool LoopClosing::PassesGeometryChecks(const LoopMatchInput& input, const Mat4f& final_pose) const {
  pcl::KdTreeFLANN<PointType> tree;
  tree.setInputCloud(input.target_cloud);
  const auto final = EvaluateGeometry(tree, *input.source_cloud, final_pose);
  const double final_rmse = std::sqrt(final.mean_squared_error);

  // Compare gravity in IMU axes: installation tilt and world yaw are unrestricted.
  const Vec3d initial_gravity =
      input.initial_source_world_pose.block<3, 3>(0, 0).cast<double>().transpose() * Vec3d::UnitZ();
  const Vec3d final_gravity = final_pose.block<3, 3>(0, 0).cast<double>().transpose() * Vec3d::UnitZ();
  const double gravity_direction_change_deg =
      std::acos(std::clamp(initial_gravity.normalized().dot(final_gravity.normalized()), -1.0, 1.0)) *
      180.0 / M_PI;
  const bool accepted = final.overlap_ratio >= options_.min_overlap_ratio_ &&
                        final_rmse <= options_.max_registration_rmse_ &&
                        gravity_direction_change_deg <= options_.max_gravity_direction_change_deg_;
  if (!accepted && options_.verbose_) {
    const auto initial =
        EvaluateGeometry(tree, *input.source_cloud, input.initial_source_world_pose);
    const double initial_rmse = std::sqrt(initial.mean_squared_error);
    LOG(INFO) << "Rejected loop " << input.candidate.idx1_ << " -> "
              << input.candidate.idx2_
              << ": overlap=" << final.overlap_ratio
              << ", correspondence RMS=" << initial_rmse
              << " -> " << final_rmse << " m, gravity direction change="
              << gravity_direction_change_deg << " deg";
  } else if (options_.verbose_) {
    LOG(INFO) << "Accepted loop " << input.candidate.idx1_ << " -> "
              << input.candidate.idx2_ << ": overlap=" << final.overlap_ratio
              << ", correspondence RMS=" << final_rmse
              << " m, gravity direction change="
              << gravity_direction_change_deg << " deg";
  }
  return accepted;
}

}  // namespace lightning
