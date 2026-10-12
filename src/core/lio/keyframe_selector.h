#ifndef LIGHTNING_KEYFRAME_SELECTOR_H
#define LIGHTNING_KEYFRAME_SELECTOR_H

#include <cmath>

#include "core/lio/lio_result.h"

namespace lightning {

/// Selects frontend keyframes without owning backend nodes or IDs.
class KeyframeSelector {
   public:
    struct Options {
        Options() {}

        double translation_threshold = 2.0;
        double rotation_threshold = 15.0 * M_PI / 180.0;
        bool localization_mode = false;
        double localization_timeout = 2.0;
    };

    explicit KeyframeSelector(Options options = Options()) : options_(options) {}

    void SetOptions(Options options) {
        options_ = options;
        Reset();
    }

    /// Returns true and records the state when a new keyframe is selected.
    bool Select(const LIOState& state);

    void Reset();
    bool HasSelected() const { return has_last_state_; }

   private:
    Options options_;
    bool has_last_state_ = false;
    LIOState last_state_;
};

}  // namespace lightning

#endif  // LIGHTNING_KEYFRAME_SELECTOR_H
