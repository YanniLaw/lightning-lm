// Copyright 2026
//
// Keyframe collection and lookup utilities.

#ifndef LIGHTNING_KEYFRAME_COLLECTION_H
#define LIGHTNING_KEYFRAME_COLLECTION_H

#include <cstddef>
#include <unordered_map>
#include <vector>

#include "common/keyframe.h"

namespace lightning {

// Keeps keyframes in insertion order and provides efficient ID lookup.
class KeyframeCollection {
   public:
    void Add(Keyframe::Ptr keyframe);

    Keyframe::Ptr Find(unsigned long id) const;
    std::size_t FindIndex(unsigned long id) const;

    const std::vector<Keyframe::Ptr>& GetAll() const { return keyframes_; }
    std::size_t Size() const { return keyframes_.size(); }

   private:
    std::vector<Keyframe::Ptr> keyframes_;
    std::unordered_map<unsigned long, Keyframe::Ptr> keyframes_by_id_;
    std::unordered_map<unsigned long, std::size_t> keyframe_indices_;
};

}  // namespace lightning

#endif  // LIGHTNING_KEYFRAME_COLLECTION_H
