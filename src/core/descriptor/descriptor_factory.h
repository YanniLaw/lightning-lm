// Copyright 2026

#ifndef SRC_CORE_DESCRIPTOR_DESCRIPTOR_FACTORY_H_
#define SRC_CORE_DESCRIPTOR_DESCRIPTOR_FACTORY_H_

#include <memory>
#include <string>

#include "core/descriptor/loop_descriptor.h"

namespace YAML {
class Node;
}

namespace lightning {

// "none" returns nullptr without parsing settings. Other unsupported names or
// invalid selected settings log an error and return nullptr.
std::unique_ptr<LoopDescriptor> CreateLoopDescriptor(
    const std::string& type, const YAML::Node& descriptor_config);

}  // namespace lightning

#endif  // SRC_CORE_DESCRIPTOR_DESCRIPTOR_FACTORY_H_
