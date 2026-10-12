#include "core/lio/keyframe_selector.h"

namespace lightning {

bool KeyframeSelector::Select(const LIOState& state) {
    if (!state.pose_is_valid || !std::isfinite(state.timestamp) || !state.pose.matrix().allFinite()) {
        return false;
    }

    if (!has_last_state_) {
        last_state_ = state;
        has_last_state_ = true;
        return true;
    }

    const SE3 delta = last_state_.pose.inverse() * state.pose;
    const bool moved_far_enough =
        delta.translation().norm() > options_.translation_threshold ||
        delta.so3().log().norm() > options_.rotation_threshold;
    const bool localization_timeout =
        options_.localization_mode && state.timestamp - last_state_.timestamp > options_.localization_timeout;
    if (!moved_far_enough && !localization_timeout) {
        return false;
    }

    last_state_ = state;
    return true;
}

void KeyframeSelector::Reset() {
    has_last_state_ = false;
    last_state_ = LIOState();
}

}  // namespace lightning
