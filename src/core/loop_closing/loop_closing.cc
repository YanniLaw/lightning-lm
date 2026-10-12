// Copyright 2026
//
// Loop candidate detection and constraint construction.

#include "core/loop_closing/loop_closing.h"

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <optional>
#include <utility>

#include "core/pose_graph/keyframe_collection.h"
#include "utils/pointcloud_utils.h"
#include "utils/thread_pool.h"

namespace lightning {

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

void LoopClosing::SetThreadPool(ThreadPoolInterface* thread_pool) {
    std::lock_guard<std::mutex> lock(match_mutex_);
    thread_pool_ = thread_pool;
}

void LoopClosing::ScheduleMatches(std::vector<LoopMatchInput> inputs) {
    std::lock_guard<std::mutex> lock(match_mutex_);
    CHECK(thread_pool_ != nullptr);
    CHECK(!done_callback_);

    try {
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
    } catch (const std::exception& exception) {
        LOG(FATAL) << "failed to construct loop-match task dependencies: " << exception.what();
    } catch (...) {
        LOG(FATAL) << "failed to construct loop-match task dependencies";
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
        try {
            for (const auto& match : match_results_) {
                if (match && match->ndt_score_ > options_.ndt_score_th_) {
                    match_result.constraints.emplace_back(*match);
                }
            }
        } catch (const std::exception& exception) {
            LOG(FATAL) << "failed to collect loop-match results: " << exception.what();
        } catch (...) {
            LOG(FATAL) << "failed to collect loop-match results with an unknown exception";
        }
        LOG(INFO) << match_results_.size() << " computations resulted in "
                  << match_result.constraints.size() << " additional constraints.";
        match_results_.clear();
        done_callback_ = nullptr;
    }

    try {
        done_callback(std::move(match_result));
    } catch (const std::exception& exception) {
        LOG(FATAL) << "loop-match completion callback failed: " << exception.what();
    } catch (...) {
        LOG(FATAL) << "loop-match completion callback failed with an unknown exception";
    }
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
        if (result && result->ndt_score_ > options_.ndt_score_th_) {
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

    const auto candidates = DetectLoopCandidates(current, keyframes);
    if (options_.verbose_) {
        LOG(INFO) << "lc: get kf " << current->GetID() << " candi: " << candidates.size();
    }

    std::vector<LoopMatchInput> inputs;
    inputs.reserve(candidates.size());
    for (const auto& candidate : candidates) {
        const auto input = BuildMatchInput(keyframes, candidate);
        if (input) {
            inputs.emplace_back(*input);
        }
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

    if (last_loop_kf_ == nullptr) {
        last_loop_kf_ = current;
        return candidates;
    }

    ++keyframes_since_detection_;
    if (keyframes_since_detection_ < options_.detection_interval_) {
        return candidates;
    }
    keyframes_since_detection_ = 0;

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
    Mat4f target_pose = input.initial_source_world_pose;
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

        rough_map1 = VoxelGrid(input.target_cloud, resolution * 0.1);
        rough_map2 = VoxelGrid(input.source_cloud, resolution * 0.1);
        ndt.setInputTarget(rough_map1);
        ndt.setInputSource(rough_map2);

        ndt.align(*output, target_pose);
        target_pose = ndt.getFinalTransformation();
        candidate.ndt_score_ = ndt.getTransformationProbability();
        if (!target_pose.allFinite() || !std::isfinite(candidate.ndt_score_)) {
            return std::nullopt;
        }
    }

    const Mat4d transformation = target_pose.cast<double>();
    if (!transformation.allFinite()) {
        return std::nullopt;
    }
    Quatd quaternion(transformation.block<3, 3>(0, 0));
    quaternion.normalize();
    const Vec3d translation = transformation.block<3, 1>(0, 3);
    candidate.Tij_ = input.reference_world_pose.inverse() * SE3(quaternion, translation);
    return candidate;
}

}  // namespace lightning
