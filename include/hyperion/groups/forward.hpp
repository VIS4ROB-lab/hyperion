/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <sym/pose2.h>
#include <sym/pose3.h>
#include <sym/rot2.h>
#include <sym/rot3.h>

#include <ostream>

#include "hyperion/forward.hpp"

namespace hyperion {

using Dim = std::int32_t;

using R1 = Vector<1>;
using R2 = Vector<2>;
using R3 = Vector<3>;
using R4 = Vector<4>;
using R5 = Vector<5>;
using R6 = Vector<6>;

using M11 = Matrix<1>;
using M22 = Matrix<2>;
using M33 = Matrix<3>;
using M44 = Matrix<4>;
using M55 = Matrix<5>;
using M66 = Matrix<6>;

using Rot2 = sym::Rot2<Scalar>;
using Rot3 = sym::Rot3<Scalar>;
using Pose2 = sym::Pose2<Scalar>;
using Pose3 = sym::Pose3<Scalar>;

enum class FrameType : bool { GLOBAL, LOCAL };

inline auto isGlobalFrame(const FrameType& frame) -> bool {
  return frame == FrameType::GLOBAL;
}

inline auto isLocalFrame(const FrameType& frame) -> bool {
  return frame == FrameType::LOCAL;
}

template <typename TGroup>
struct GroupOps;

template <int AmbientDim, int TangentDim>
struct ConstexprGroupDims {
  ConstexprGroupDims() = delete;
  static std::integral_constant<int, AmbientDim> kAmbientDim;
  static std::integral_constant<int, TangentDim> kTangentDim;
};

struct GroupDims {
  /// Equality operator.
  /// @param rhs Right-hand side input.
  /// @return True if equal.
  bool operator==(const GroupDims& rhs) const { return ambient == rhs.ambient && tangent == rhs.tangent; }

  /// Plus operator.
  /// @param rhs Right-hand side.
  /// @return Sum of this and rhs.
  auto operator+(const GroupDims& rhs) const -> GroupDims { return {ambient + rhs.ambient, tangent + rhs.tangent}; }

  /// Plus operator (in-place).
  /// @param rhs Right-hand side.
  /// @return Sum of this and rhs (in-place).
  auto operator+=(const GroupDims& rhs) -> GroupDims& {
    ambient += rhs.ambient;
    tangent += rhs.tangent;
    return *this;
  }

  friend auto operator<<(std::ostream& os, const GroupDims& group_dims) -> std::ostream& {
    return os << "<" << group_dims.ambient << ", " << group_dims.tangent << ">";
  }

  Dim ambient{0};  ///< Ambient dimension.
  Dim tangent{0};  ///< Tangent dimension.
};

}  // namespace hyperion
