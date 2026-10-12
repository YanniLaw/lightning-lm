// Copyright 2026
//
// Graph optimizer factory.

#ifndef SRC_CORE_OPTIMIZATION_OPTIMIZER_FACTORY_H_
#define SRC_CORE_OPTIMIZATION_OPTIMIZER_FACTORY_H_

#include <memory>
#include <string>

#include "core/optimization/graph_optimizer.h"

namespace lightning {

// Returns nullptr for unsupported types; never silently selects a fallback.
std::unique_ptr<GraphOptimizer> CreateGraphOptimizer(const std::string& type);

}  // namespace lightning

#endif  // SRC_CORE_OPTIMIZATION_OPTIMIZER_FACTORY_H_
