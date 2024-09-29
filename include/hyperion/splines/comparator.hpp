/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <glog/logging.h>

#include "hyperion/forward.hpp"
#include "hyperion/groups/groups.hpp"
#include "hyperion/utils/clock.hpp"

namespace hyperion::splines {

namespace internal {

template <typename TGroup>
struct ComparatorHelper;

template <typename TScalar>
struct ComparatorHelper<sym::Pose3<TScalar>> {
  // Typedefs
  using Group = sym::Pose3<TScalar>;
  using Tangent = sym::Pose3<TScalar>::TangentVec;

  struct Error {
    TScalar rotation_error;
    TScalar position_error;
  };

  static auto ComputeErrors(const std::vector<Tangent>& diffs) -> std::vector<Error> {
    CHECK(!diffs.empty());
    std::vector<Error> errors;
    errors.reserve(diffs.size());
    for (const auto& diff : diffs) {
      const auto error = {
          .rotation_error = diff.template head<3>().norm(),
          .position_error = diff.template tail<3>().norm()};
      errors.emplace_back(error);
    }
    return errors;
  }

  static auto ComputeRMSE(const std::vector<Tangent>& diffs) -> Error {
    CHECK(!diffs.empty());
    Error error = {0, 0};
    for (const auto& diff : diffs) {
      error.rotation_error += diff.template head<3>().squaredNorm();
      error.position_error += diff.template tail<3>().squaredNorm();
    }

    const auto num_diffs = diffs.size();
    return {
        .rotation_error = std::sqrt(error.rotation_error / num_diffs),
        .position_error = std::sqrt(error.position_error / num_diffs)};
  }
};

}  // namespace internal

class Comparator {
 public:
  template <typename TSpline>
  static auto ComputeDiffs(const TSpline& lhs, const TSpline& rhs, const Duration& dt) {
    // Typedefs
    using Group = typename TSpline::Value;
    using GroupOps = sym::LieGroupOps<Group>;
    using Tangent = typename GroupOps::TangentVec;
    using Diffs = std::vector<Tangent>;

    const auto t0 = std::max(lhs.t0(), rhs.t0());
    const auto tn = std::min(lhs.tn(), rhs.tn());

    if (tn < t0) {
      LOG(WARNING) << "Spline ranges do not overlap.";
      return Diffs{};
    }

    Diffs diffs;
    const auto num_diffs = (tn - t0) / dt + 1;
    diffs.reserve(num_diffs);

    for (auto ti = t0; ti < tn; ti += dt) {
      const auto lhs_value_i = lhs.value(ti);
      const auto rhs_value_i = rhs.value(ti);
      const auto diff_i = GroupOps::LocalCoordinates(lhs_value_i, rhs_value_i, kDefaultEpsilon);
      diffs.emplace_back(diff_i);
    }

    return diffs;
  }

  template <typename TSpline>
  static auto ComputeErrors(const TSpline& lhs, const TSpline& rhs, const Duration& dt) {
    // Typedefs
    using Group = typename TSpline::Value;
    using Helper = internal::ComparatorHelper<Group>;
    const auto diffs = ComputeDiffs(lhs, rhs, dt);
    return Helper::ComputeErrors(diffs);
  }

  template <typename TSpline>
  static auto ComputeRMSE(const TSpline& lhs, const TSpline& rhs, const Duration& dt) {
    // Typedefs
    using Group = typename TSpline::Value;
    using Helper = internal::ComparatorHelper<Group>;
    const auto diffs = ComputeDiffs(lhs, rhs, dt);
    return Helper::ComputeRMSE(diffs);
  }
};

}  // namespace hyperion::splines
