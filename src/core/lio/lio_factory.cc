#include "core/lio/lio_factory.h"

#include <glog/logging.h>

#include "core/lio/aa-fasterlio/laser_mapping.h"

namespace lightning {

std::unique_ptr<LIO> CreateLIO(const std::string& type) {
    if (type == "aa_fasterlio") {
        return std::make_unique<LaserMapping>();
    }

    LOG(ERROR) << "unknown LIO frontend type: " << type;
    return nullptr;
}

}  // namespace lightning
