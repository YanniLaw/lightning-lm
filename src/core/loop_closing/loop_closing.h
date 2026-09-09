// Copyright 2026
//
// Loop candidate detection and constraint construction.

#ifndef LIGHTNING_LOOP_CLOSING_H
#define LIGHTNING_LOOP_CLOSING_H

#include <vector>

#include "common/keyframe.h"
#include "common/loop_candidate.h"

namespace lightning {

class KeyframeCollection;

// Finds candidate loop closures and computes their relative pose constraints.
class LoopClosing {
   public:
    struct Options {
        Options() {}

        bool verbose_ = true;

        int loop_kf_gap_ = 20;
        int min_id_interval_ = 20;
        int closest_id_th_ = 50;
        double max_range_ = 30.0;
        double ndt_score_th_ = 1.0;
    };

    explicit LoopClosing(Options options = Options());

    LoopClosing(const LoopClosing&) = delete;
    LoopClosing& operator=(const LoopClosing&) = delete;

    // Applies configuration before the first constraint computation.
    void SetOptions(Options options);

    // Computes valid loop constraints synchronously for one keyframe.
    std::vector<LoopCandidate> ComputeConstraints(const Keyframe::Ptr& current,
                                                  const KeyframeCollection& keyframes);

   private:
    std::vector<LoopCandidate> DetectLoopCandidates(const Keyframe::Ptr& current,
                                                    const KeyframeCollection& keyframes);

    void ComputeLoopCandidates(const KeyframeCollection& keyframes,
                               std::vector<LoopCandidate>& candidates);

    void ComputeForCandidate(const KeyframeCollection& keyframes, LoopCandidate& candidate);

    Options options_;
    Keyframe::Ptr last_loop_kf_;
};

}  // namespace lightning

#endif  // LIGHTNING_LOOP_CLOSING_H
