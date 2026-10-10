// Copyright 2026
//
// Graph optimizer factory.

#include "core/optimization/optimizer_factory.h"

#include <glog/logging.h>

#include "core/optimization/miao_optimizer.h"

namespace lightning {

std::unique_ptr<GraphOptimizer> CreateGraphOptimizer(const std::string& type) {
    if (type == "miao") {
        return std::make_unique<MiaoOptimizer>();
    }
    LOG(ERROR) << "Unsupported graph optimizer type: " << type << "; supported type: miao";
    return nullptr;
}

}  // namespace lightning
