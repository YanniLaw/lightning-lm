// Copyright 2026
//
// Keyframe collection and lookup utilities.

#include "core/loop_closing/keyframe_collection.h"

namespace lightning {

void KeyframeCollection::Add(Keyframe::Ptr keyframe) {
    keyframes_.emplace_back(keyframe);
    keyframes_by_id_[keyframe->GetID()] = keyframe;
    keyframe_indices_[keyframe->GetID()] = keyframes_.size() - 1;
}

Keyframe::Ptr KeyframeCollection::Find(unsigned long id) const {
    const auto it = keyframes_by_id_.find(id);
    return it == keyframes_by_id_.end() ? nullptr : it->second;
}

std::size_t KeyframeCollection::FindIndex(unsigned long id) const {
    const auto it = keyframe_indices_.find(id);
    return it == keyframe_indices_.end() ? keyframes_.size() : it->second;
}

}  // namespace lightning
