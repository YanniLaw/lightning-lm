#ifndef LIGHTNING_SCAN_ACCUMULATOR_H
#define LIGHTNING_SCAN_ACCUMULATOR_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <list>
#include <memory>

#include "core/lio/lio_result.h"

namespace lightning {

/// Maintains the small frontend scan history used for projected clouds.
///
/// Entries own immutable copies of the selected scan.  The accumulator never
/// assigns a backend ID and never writes to an input result cloud.
class ScanAccumulator {
   public:
    struct Options {
        Options() {}

        std::size_t max_scans = 5;
        std::size_t max_points_per_scan = 1000;
        double replacement_translation_threshold = 3.0;
        double replacement_rotation_threshold = 20.0 * M_PI / 180.0;
    };

    explicit ScanAccumulator(Options options = Options()) : options_(options) {}

    void SetOptions(Options options) {
        options_ = options;
        TrimToLimit();
    }

    /// Adds the selected result and applies the original history replacement rule.
    void AddSelected(const LIOResult& result);

    /// Builds a fresh current-frame cloud followed by projected history points.
    CloudPtr BuildProjectedCloud(const LIOResult& current) const;

    void Reset() { selected_scans_.clear(); }
    std::size_t Size() const { return selected_scans_.size(); }

   private:
    struct Entry {
        LIOState state;
        SE3 body_from_lidar = SE3();
        std::shared_ptr<const PointCloudType> cloud;
    };

    void TrimToLimit();

    Options options_;
    std::list<Entry> selected_scans_;
};

}  // namespace lightning

#endif  // LIGHTNING_SCAN_ACCUMULATOR_H
