/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyperion/forward.hpp"

namespace hyperion {

class Vertex;

class Graph;
struct GraphOptions;

class Solver;
struct SolverOptions;
struct SolverSummary;

enum class VertexType : std::uint8_t { NODE, FACTOR };
enum class ValueType : std::uint8_t { CONSTANT, CONSTANT_GAUSSIAN, GAUSSIAN };
enum class Ownership : std::uint8_t { TAKE_OWNERSHIP, DO_NOT_TAKE_OWNERSHIP };

}  // namespace hyperion
