/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <cstdint>

namespace hyperion::splines {

enum class IntervalType : std::uint8_t { UNIFORM, NON_UNIFORM };

enum class SplineType : std::uint8_t { BSPLINE, ZSPLINE };

}  // namespace hyperion::splines
