#ifndef LIGHTNING_LIO_RESULT_CONVERSION_H
#define LIGHTNING_LIO_RESULT_CONVERSION_H

#include "common/nav_state.h"
#include "core/lio/lio_result.h"

namespace lightning {

/// Converts the transport-independent LIO state for legacy consumers.
bool ConvertToNavState(const LIOState& state, NavState& output);

}  // namespace lightning

#endif  // LIGHTNING_LIO_RESULT_CONVERSION_H
