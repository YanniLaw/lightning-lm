#include "core/lio/lio_result_conversion.h"

#include <cmath>

namespace lightning {

bool ConvertToNavState(const LIOState& state, NavState& output) {
    output = NavState();
    if (!state.pose_is_valid || !std::isfinite(state.timestamp) || !state.pose.matrix().allFinite()) {
        output.pose_is_ok_ = false;
        return false;
    }

    output.timestamp_ = state.timestamp;
    output.SetPose(state.pose);
    output.pose_is_ok_ = true;
    output.lidar_odom_reliable_ = state.lidar_odom_reliable;

    if (state.velocity.has_value() && state.velocity->allFinite()) {
        output.SetVel(*state.velocity);
    }
    if (state.gyro_bias.has_value() && state.gyro_bias->allFinite()) {
        output.bg_ = *state.gyro_bias;
    }
    if (state.gravity.has_value() && state.gravity->allFinite()) {
        output.grav_ = *state.gravity;
    }
    return true;
}

}  // namespace lightning
