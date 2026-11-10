// Copyright 2026
//
// Pose graph maintenance and optimization.

#include "core/pose_graph/pose_graph.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <unordered_map>
#include <utility>

#include "core/lio/lio_result_conversion.h"
#include "core/descriptor/descriptor_factory.h"
#include "core/optimization/optimizer_factory.h"
#include "utils/thread_pool.h"

namespace lightning {
namespace {

bool LoadDescriptorOptions(const YAML::Node& config, LoopClosing::Options* options) {
  try {
    const YAML::Node loop_config = config ? config["loop_closing"] : YAML::Node();
    if (loop_config && loop_config["descriptor_type"]) {
      options->descriptor_type_ = loop_config["descriptor_type"].as<std::string>();
    }
    if (options->descriptor_type_ == "none") {
      return true;
    }
    const YAML::Node descriptors = config ? config["descriptor"] : YAML::Node();
    const YAML::Node node =
        descriptors ? descriptors[options->descriptor_type_] : YAML::Node();
    if (node) {
      if (node["sub_frame_num"]) {
        options->descriptor_submap_keyframes_ = node["sub_frame_num"].as<int>();
      }
      if (node["ds_size"]) {
        options->descriptor_downsample_resolution_ = node["ds_size"].as<double>();
      }
      if (node["skip_near_num"]) {
        options->descriptor_skip_near_num_ = node["skip_near_num"].as<int>();
      }
    }
  } catch (const YAML::Exception& exception) {
    LOG(ERROR) << "invalid loop descriptor configuration: " << exception.what();
    return false;
  }
  if (options->descriptor_submap_keyframes_ < 1 || options->descriptor_skip_near_num_ < 0 ||
      !std::isfinite(options->descriptor_downsample_resolution_) ||
      options->descriptor_downsample_resolution_ <= 0.0) {
    LOG(ERROR) << "invalid common loop descriptor settings";
    return false;
  }
  return true;
}

}  // namespace

PoseGraph::PoseGraph(Options options) : options_(std::move(options)) {}

PoseGraph::~PoseGraph() { Stop(); }

bool PoseGraph::Init(const std::string& yaml_path) {
    if (initialized_) {
        LOG(WARNING) << "pose graph is already initialized";
        return true;
    }

    YAML::Node config;
    if (!yaml_path.empty()) {
        config = YAML::LoadFile(yaml_path);
    }

    const YAML::Node system_config = config["system"];
    if (system_config && system_config["optimizer_type"]) {
        options_.optimizer_type_ = system_config["optimizer_type"].as<std::string>();
    }

    auto& loop_options = options_.loop_closing_;
    const YAML::Node loop_config = config["loop_closing"];
    if (loop_config) {
        if (loop_config["loop_kf_gap"]) {
            // Keep accepting the legacy key as an alias for detection_interval.
            loop_options.detection_interval_ = loop_config["loop_kf_gap"].as<int>();
        }
        if (loop_config["detection_interval"]) {
            loop_options.detection_interval_ = loop_config["detection_interval"].as<int>();
        }
        if (loop_config["optimize_every_n_nodes"]) {
            options_.optimize_every_n_nodes_ = loop_config["optimize_every_n_nodes"].as<int>();
        }
        if (loop_config["num_threads"]) {
            options_.num_threads_ = loop_config["num_threads"].as<int>();
        }
        if (loop_config["min_id_interval"]) {
            loop_options.min_id_interval_ = loop_config["min_id_interval"].as<int>();
        }
        if (loop_config["closest_id_th"]) {
            loop_options.closest_id_th_ = loop_config["closest_id_th"].as<int>();
        }
        if (loop_config["max_range"]) {
            loop_options.max_range_ = loop_config["max_range"].as<double>();
        }
        if (loop_config["ndt_score_th"]) {
            loop_options.ndt_score_th_ = loop_config["ndt_score_th"].as<double>();
        }
        if (loop_config["min_overlap_ratio"]) {
            loop_options.min_overlap_ratio_ = loop_config["min_overlap_ratio"].as<double>();
        }
        if (loop_config["max_registration_rmse"]) {
            loop_options.max_registration_rmse_ = loop_config["max_registration_rmse"].as<double>();
        }
        if (loop_config["max_gravity_direction_change_deg"]) {
            loop_options.max_gravity_direction_change_deg_ =
                loop_config["max_gravity_direction_change_deg"].as<double>();
        }
        if (loop_config["with_height"]) {
            options_.with_height_ = loop_config["with_height"].as<bool>();
        }
    }

    if (options_.enable_loop_closing_ && !LoadDescriptorOptions(config, &loop_options)) {
        LOG(ERROR) << "Failed to load descriptor options";
        return false;
    }
    if (options_.num_threads_ <= 0 || options_.optimize_every_n_nodes_ <= 0 ||
        (options_.enable_loop_closing_ && loop_options.descriptor_type_ == "none" &&
         loop_options.detection_interval_ <= 0)) {
        LOG(ERROR) << "invalid pose graph scheduling options";
        return false;
    }
    if (options_.enable_loop_closing_ &&
        (!std::isfinite(loop_options.min_overlap_ratio_) || loop_options.min_overlap_ratio_ < 0.0 ||
         loop_options.min_overlap_ratio_ > 1.0 || !std::isfinite(loop_options.max_registration_rmse_) ||
         loop_options.max_registration_rmse_ <= 0.0 ||
         !std::isfinite(loop_options.max_gravity_direction_change_deg_) ||
         loop_options.max_gravity_direction_change_deg_ <= 0.0 ||
         loop_options.max_gravity_direction_change_deg_ > 180.0)) {
        LOG(ERROR) << "invalid loop geometry thresholds";
        return false;
    }

    if (options_.enable_loop_closing_) {
        const YAML::Node descriptor_config =
            loop_options.descriptor_type_ != "none" ? config["descriptor"] : YAML::Node();
        auto descriptor = CreateLoopDescriptor(loop_options.descriptor_type_, descriptor_config);
        if (loop_options.descriptor_type_ != "none" && !descriptor) {
            LOG(ERROR) << "Failed to create loop descriptor";
            return false;
        }
        loop_options.verbose_ = options_.verbose_;
        loop_closing_.SetOptions(loop_options);
        loop_closing_.SetDescriptor(std::move(descriptor));
        LOG(INFO) << "loop descriptor: " << loop_options.descriptor_type_;

        optimizer_ = CreateGraphOptimizer(options_.optimizer_type_);
        if (!optimizer_) {
            LOG(ERROR) << "Failed to create graph optimizer";
            return false;
        }
        LOG(INFO) << "pose graph optimizer: " << options_.optimizer_type_;
    }

    info_motion_.setIdentity();
    info_motion_.block<3, 3>(0, 0) =
        Mat3d::Identity() * 1.0 / (options_.motion_trans_noise_ * options_.motion_trans_noise_);
    info_motion_.block<3, 3>(3, 3) =
        Mat3d::Identity() * 1.0 / (options_.motion_rot_noise_ * options_.motion_rot_noise_);

    info_loops_.setIdentity();
    info_loops_.block<3, 3>(0, 0) = Mat3d::Identity() * 1.0 / (options_.loop_trans_noise_ * options_.loop_trans_noise_);
    info_loops_.block<3, 3>(3, 3) = Mat3d::Identity() * 1.0 / (options_.loop_rot_noise_ * options_.loop_rot_noise_);

    thread_pool_ = std::make_unique<ThreadPool>(options_.num_threads_);
    loop_closing_.SetThreadPool(thread_pool_.get());

    initialized_ = true;
    accepting_.store(true);
    return true;
}

Keyframe::Ptr PoseGraph::AddKeyframe(const LIOResult& result) {
    if (result.update_type != LIOUpdateType::kScanMatched || !result.keyframe_selected || !result.state.pose_is_valid ||
        !result.cloud || result.cloud->empty()) {
        return nullptr;
    }

    for (const PointType& point : result.cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            !std::isfinite(point.intensity)) {
            return nullptr;
        }
    }

    NavState state;
    if (!ConvertToNavState(result.state, state)) {
        return nullptr;
    }

    Keyframe::Ptr keyframe;
    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        if (!initialized_ || !accepting_.load() || stopped_.load() ||
            result.state.timestamp <= last_accepted_timestamp_) {
            return nullptr;
        }

        keyframe = std::make_shared<Keyframe>(static_cast<unsigned long>(next_keyframe_id_++), result.cloud, state);
        last_accepted_timestamp_ = result.state.timestamp;


        // Keep acceptance and queue insertion in one critical section.  This
        // prevents Stop() from placing its flush barrier before a keyframe
        // that was accepted just before shutdown began.
        AddWorkItem([this, keyframe]() { return HandleKeyframe(keyframe); });
    }
    return keyframe;
}

std::vector<KeyframeMapEntry> PoseGraph::GetMapSnapshot() const {
    std::lock_guard<std::mutex> graph_lock(graph_mutex_);

    const auto& committed_keyframes = keyframes_.GetAll();

    std::vector<KeyframeMapEntry> snapshot;
    snapshot.reserve(committed_keyframes.size());
    for (const auto& keyframe : committed_keyframes) {
        if (!keyframe || !keyframe->GetCloud()) {
            continue;
        }

        KeyframeMapEntry entry;
        entry.id = keyframe->GetID();
        entry.timestamp = keyframe->GetState().timestamp_;
        entry.cloud = keyframe->GetCloud();
        entry.lio_pose = keyframe->GetLIOPose();
        entry.optimized_pose = keyframe->GetOptPose();
        snapshot.emplace_back(std::move(entry));
    }
    return snapshot;
}

bool PoseGraph::FlushAndGetMapSnapshot(std::vector<KeyframeMapEntry>* snapshot) {
    if (snapshot == nullptr) {
        return false;
    }
    return Flush(snapshot);
}

bool PoseGraph::Flush(std::vector<KeyframeMapEntry>* snapshot) {
    if (!initialized_) {
        return false;
    }

    std::lock_guard<std::mutex> flush_lock(flush_mutex_);
    if (stopped_.load()) {
        if (snapshot != nullptr) {
            *snapshot = GetMapSnapshot();
        }
        return true;
    }

    const auto flush_state = std::make_shared<FlushState>();
    AddWorkItem([this, flush_state, snapshot]() {
        // Insert the completion before any work accepted after this barrier.
        // DrainWorkQueue resumes it only after WhenDone and optimization finish.
        std::lock_guard<std::mutex> lock(work_queue_mutex_);
        work_queue_->push_front({std::chrono::steady_clock::now(), [this, flush_state, snapshot]() {
                                     std::vector<KeyframeMapEntry> map_snapshot;
                                     if (snapshot != nullptr) {
                                         map_snapshot = GetMapSnapshot();
                                     }
                                     CompleteFlush(flush_state, std::move(map_snapshot));
                                     return WorkItem::Result::kDoNotRunOptimization;
                                 }});
        return WorkItem::Result::kRunOptimization;
    });

    std::unique_lock<std::mutex> lock(flush_state->mutex);
    flush_state->condition.wait(lock, [&flush_state]() { return flush_state->done; });
    if (snapshot != nullptr) {
        *snapshot = std::move(flush_state->snapshot);
    }
    return true;
}

void PoseGraph::Stop() {
    {
        std::unique_lock<std::mutex> lock(stop_mutex_);
        if (stopped_.load()) {
            return;
        }
        if (stop_in_progress_) {
            stop_condition_.wait(lock, [this]() { return stopped_.load(); });
            return;
        }
        stop_in_progress_ = true;
    }

    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        accepting_.store(false);
    }

    try {
        Flush(nullptr);
    } catch (const std::exception& exception) {
        LOG(FATAL) << "pose graph final flush failed: " << exception.what();
    } catch (...) {
        LOG(FATAL) << "pose graph final flush failed with an unknown exception";
    }
    thread_pool_.reset();
    loop_closing_.SetThreadPool(nullptr);

    {
        std::lock_guard<std::mutex> lock(stop_mutex_);
        stopped_.store(true);
        stop_in_progress_ = false;
    }
    stop_condition_.notify_all();
}

void PoseGraph::AddWorkItem(std::function<WorkItem::Result()> task) {
    std::lock_guard<std::mutex> lock(work_queue_mutex_);
    if (!work_queue_) {
        work_queue_ = std::make_unique<WorkQueue>();
        CHECK(thread_pool_ != nullptr);
        auto drain_task = std::make_unique<Task>();
        drain_task->SetWorkItem([this]() { DrainWorkQueue(); });
        thread_pool_->Schedule(std::move(drain_task));
    }

    work_queue_->push_back({std::chrono::steady_clock::now(), std::move(task)});
}

void PoseGraph::DrainWorkQueue() {
    bool process_work_queue = true;
    std::size_t work_queue_size = 0;
    while (process_work_queue) {
        std::function<WorkItem::Result()> work_item;
        {
            std::lock_guard<std::mutex> lock(work_queue_mutex_);
            if (!work_queue_ || work_queue_->empty()) {
                work_queue_.reset();
                return;
            }
            work_item = std::move(work_queue_->front().task);
            work_queue_->pop_front();
            work_queue_size = work_queue_->size();
        }

        try {
            process_work_queue =
                work_item() == WorkItem::Result::kDoNotRunOptimization;
        } catch (const std::exception& exception) {
            LOG(FATAL) << "Pose graph work item failed: " << exception.what();
        } catch (...) {
            LOG(FATAL) << "Pose graph work item failed with an unknown exception";
        }
    }

    LOG_IF(INFO, work_queue_size > 0) << "Remaining work items in queue: " << work_queue_size;
    // WhenDone also schedules an empty result when loop closing is disabled.
    // Resume asynchronously so flush barriers do not recurse into queue draining.
    loop_closing_.WhenDone([this](LoopClosing::MatchResult result) {
        HandleWorkQueue(std::move(result));
    });
}

WorkItem::Result PoseGraph::HandleKeyframe(const Keyframe::Ptr& keyframe) {
    DataCallback data_callback;
    PoseGraphDataPtr data_snapshot;
    bool should_optimize = false;
    {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        keyframe->SetOptPose(map_from_local_ * keyframe->GetLIOPose());
        keyframes_.Add(keyframe);
        if (!AddKeyframeToGraph(keyframe)) {
            LOG(FATAL) << "failed to add keyframe to pose graph: " << keyframe->GetID();
        }

        if (options_.verbose_) {
            LOG(INFO) << "Add keyframe " << keyframe->GetID()
                      << ", pos: [" << std::setprecision(3)
                      << keyframe->GetLIOPose().translation().transpose()
                      << " ], time: " << std::fixed << std::setprecision(4)
                      << keyframe->GetState().timestamp_
                      << ", points: " << keyframe->GetCloud()->size();

        }

        ++unoptimized_nodes_;
        data_callback = data_callback_;
        if (data_callback) {
            data_snapshot = CreateDataSnapshot(false);
        }
        should_optimize = options_.enable_loop_closing_ &&
                          unoptimized_nodes_ >= static_cast<std::size_t>(options_.optimize_every_n_nodes_);
    }

    if (options_.enable_loop_closing_) {
        // Keyframes and poses are stable within this serial work item. Keep
        // descriptor preparation outside the mutex used by map snapshots.
        auto inputs = loop_closing_.PrepareMatches(keyframe, keyframes_);
        loop_closing_.ScheduleMatches(std::move(inputs));
        loop_closing_.NotifyEndOfNode();
    }

    if (data_callback) {
        try {
            data_callback(std::move(data_snapshot));
        } catch (const std::exception& exception) {
            LOG(ERROR) << "pose graph data callback failed: " << exception.what();
        } catch (...) {
            LOG(ERROR) << "pose graph data callback failed with an unknown exception";
        }
    }

    return should_optimize ? WorkItem::Result::kRunOptimization : WorkItem::Result::kDoNotRunOptimization;
}

void PoseGraph::HandleWorkQueue(LoopClosing::MatchResult result) {
    OptimizedCallback optimized_callback;
    DataCallback data_callback;
    PoseGraphDataPtr data_snapshot;
    bool optimized = false;
    std::size_t loop_count = 0;

    {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        AddLoopConstraints(result.constraints);
        optimized = OptimizeGraph();

        loop_count = loop_constraints_.size();
        optimized_callback = optimized_callback_;
        data_callback = data_callback_;
        if (data_callback && optimized) {
            data_snapshot = CreateDataSnapshot(true);
        }
    }

    if (optimized && optimized_callback) {
        try {
            optimized_callback();
        } catch (const std::exception& exception) {
            LOG(ERROR) << "pose graph optimized callback failed: " << exception.what();
        } catch (...) {
            LOG(ERROR) << "pose graph optimized callback failed with an unknown exception";
        }
    }
    if (optimized) {
        LOG(INFO) << "pose graph optimize finished, loops: " << loop_count;
    }
    if (data_callback && data_snapshot) {
        try {
            data_callback(std::move(data_snapshot));
        } catch (const std::exception& exception) {
            LOG(ERROR) << "pose graph data callback failed: " << exception.what();
        } catch (...) {
            LOG(ERROR) << "pose graph data callback failed with an unknown exception";
        }
    }

    unoptimized_nodes_ = 0;

    DrainWorkQueue();
}

void PoseGraph::CompleteFlush(const std::shared_ptr<FlushState>& flush_state,
                              std::vector<KeyframeMapEntry> snapshot) {
    {
        std::lock_guard<std::mutex> lock(flush_state->mutex);
        flush_state->snapshot = std::move(snapshot);
        flush_state->done = true;
    }
    flush_state->condition.notify_all();
}

bool PoseGraph::AddKeyframeToGraph(const Keyframe::Ptr& current) {
    if (!optimizer_) {
        return true;
    }

    const GraphNode node{current->GetID(), current->GetOptPose(), false};
    if (!optimizer_->AddNode(node)) {
        LOG(ERROR) << "failed to add pose graph node: " << current->GetID();
        return false;
    }

    bool added_constraint = false;
    const std::size_t current_index = keyframes_.FindIndex(current->GetID());
    const auto& all_keyframes = keyframes_.GetAll();
    for (std::size_t i = 1; i < 3 && i <= current_index; ++i) {
        const auto& last_keyframe = all_keyframes[current_index - i];
        RelativeConstraint motion;
        motion.id = next_constraint_id_++;
        motion.from_id = last_keyframe->GetID();
        motion.to_id = current->GetID();
        motion.measurement = last_keyframe->GetLIOPose().inverse() * current->GetLIOPose();
        motion.information = info_motion_;
        if (!optimizer_->AddRelativeConstraint(motion)) {
            LOG(ERROR) << "failed to add motion constraint: " << motion.id;
            return false;
        }
        added_constraint = true;
    }

    if (options_.with_height_) {
        HeightConstraint height;
        height.id = next_constraint_id_++;
        height.node_id = current->GetID();
        height.information = 1.0 / (options_.height_noise_ * options_.height_noise_);
        if (!optimizer_->AddHeightConstraint(height)) {
            LOG(ERROR) << "failed to add height constraint: " << height.id;
            return false;
        }
        added_constraint = true;
    }
    graph_dirty_ = graph_dirty_ || added_constraint;
    return true;
}

void PoseGraph::AddLoopConstraints(const std::vector<LoopCandidate>& constraints) {
    if (!optimizer_) {
        return;
    }

    for (const auto& constraint : constraints) {
        const auto pair = std::make_pair(constraint.idx1_, constraint.idx2_);
        if (loop_pairs_.count(pair) != 0U) {
            continue;
        }

        RelativeConstraint loop;
        loop.id = next_constraint_id_++;
        loop.from_id = constraint.idx1_;
        loop.to_id = constraint.idx2_;
        loop.measurement = constraint.Tij_;
        loop.information = info_loops_;
        loop.is_loop = true;
        loop.robust_delta = options_.rk_loop_th_;
        if (!optimizer_->AddRelativeConstraint(loop)) {
            LOG(FATAL) << "failed to add loop constraint " << loop.id << " for "
                       << loop.from_id << " -> " << loop.to_id;
        }
        loop_constraints_.push_back(PoseGraphConstraint{loop.id, static_cast<unsigned long>(loop.from_id),
                                                        static_cast<unsigned long>(loop.to_id), false});
        loop_pairs_.insert(pair);
        graph_dirty_ = true;
    }
}

bool PoseGraph::OptimizeGraph() {
    if (!optimizer_ || !graph_dirty_) {
        return false;
    }
    const OptimizationResult result = optimizer_->Optimize(20);
    if (!result.usable) {
        LOG(FATAL) << "pose graph optimization failed";
    }

    Keyframe::Ptr latest_optimized;
    for (const auto& estimate : result.nodes) {
        const auto keyframe = keyframes_.Find(estimate.id);
        if (keyframe != nullptr) {
            keyframe->SetOptPose(estimate.pose);
            if (!latest_optimized || keyframe->GetID() > latest_optimized->GetID()) {
                latest_optimized = keyframe;
            }
        }
    }
    if (latest_optimized) {
        map_from_local_ = latest_optimized->GetOptPose() * latest_optimized->GetLIOPose().inverse();
    }

    std::unordered_map<std::uint64_t, bool> outlier_states;
    outlier_states.reserve(result.loops.size());
    for (const auto& loop : result.loops) {
        outlier_states.emplace(loop.id, loop.is_outlier);
    }
    for (auto& loop : loop_constraints_) {
        const auto state = outlier_states.find(loop.id);
        if (state != outlier_states.end()) {
            loop.is_outlier = state->second;
        }
    }

    if (options_.verbose_ && !loop_constraints_.empty()) {
        const auto outlier_count = std::count_if(loop_constraints_.begin(), loop_constraints_.end(),
                                                 [](const PoseGraphConstraint& loop) { return loop.is_outlier; });
        LOG(INFO) << "loop outliers: " << outlier_count << "/" << loop_constraints_.size();
    }
    graph_dirty_ = false;
    return true;
}

PoseGraphDataPtr PoseGraph::CreateDataSnapshot(bool optimized) {
    auto data = std::make_shared<PoseGraphData>();
    data->version = ++data_version_;
    data->optimized = optimized;
    data->map_from_local = map_from_local_;

    const auto& all_keyframes = keyframes_.GetAll();
    data->nodes.reserve(all_keyframes.size());
    for (const auto& keyframe : all_keyframes) {
        if (!keyframe) {
            continue;
        }

        PoseGraphNode node;
        node.id = keyframe->GetID();
        node.timestamp = keyframe->GetState().timestamp_;
        node.pose = keyframe->GetOptPose();
        data->nodes.emplace_back(std::move(node));
    }
    data->loop_constraints = loop_constraints_;
    return data;
}

}  // namespace lightning
