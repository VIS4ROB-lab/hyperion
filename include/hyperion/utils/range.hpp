/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <cmath>
#include <ostream>
#include <random>
#include <type_traits>
#include <vector>

namespace hyperion {

template <typename T>
concept Integral = std::is_integral_v<T>;
template <typename T>
concept Float = std::is_floating_point_v<T>;
template <typename T>
concept Real = Integral<T> || Float<T>;

/// Boundary policy enum.
enum class BoundaryPolicy { INCLUSIVE, LOWER_INCLUSIVE_ONLY, UPPER_INCLUSIVE_ONLY, EXCLUSIVE };

template <typename Type, BoundaryPolicy boundary_policy>
  requires Integral<Type> || Real<Type>
class Range {
 public:
  /// Retrieves the boundary policy.
  /// \return Boundary policy.
  [[nodiscard]] static constexpr auto Policy() -> BoundaryPolicy { return boundary_policy; }

  /// Returns the lower bound (considering the boundaries).
  /// \return Lower bound of this range.
  auto lowerBound() const -> Type;

  /// Returns the upper bound (considering the boundaries).
  /// \return Upper bound of this range.
  auto upperBound() const -> Type;

  /// Checks whether a type is smaller than the range.
  /// \param type Query type.
  /// \return True if type compares smaller.
  auto isSmaller(const Type& type) const -> bool;

  /// Checks whether a type is greater than the range.
  /// \param type Query type.
  /// \return True if type compares greater.
  auto isGreater(const Type& type) const -> bool;

  /// Checks whether value if contained in range.
  /// \param type Query type.
  /// \return True if type is contained.
  auto contains(const Type& type) const -> bool { return !isSmaller(type) && !isGreater(type); }

  /// Determines the span of the range.
  /// \return Span of the range (according to boundary policy).
  auto span() const -> Type;

  /// Determines if the range is empty.
  /// (i.e. lower bound is larger than upper bound).
  /// \return True if range is empty.
  [[nodiscard]] auto empty() const -> bool { return upperBound() < lowerBound(); }

  /// Samples a range by a sampling rate.
  /// \param rate Sampling rate.
  /// \return Samples.
  auto sample(const Type& rate) const -> std::vector<Type> {
    // Allocate memory;
    std::vector<Type> samples;
    samples.reserve(std::ceil(rate * span()));

    // Fetch lower bound.
    const auto lower_bound = lowerBound();
    const auto upper_bound = upperBound();
    const auto inverse_rate = Type{1} / rate;

    // Generate samples.
    auto i = std::size_t{0};
    while (true) {
      const auto sample_i = lower_bound + i * inverse_rate;
      if (sample_i <= upper_bound) {
        samples.emplace_back(sample_i);
        ++i;
      } else {
        return samples;
      }
    }
  }

  /// Retrieves a random sample contained inside the range.
  /// \return Random sample.
  template <typename TGenerator>
  auto sample(const TGenerator& generator) const -> Type;

  /// Retrieves the closest sample in the range.
  /// \param type Query type.
  /// \return Closest sample.
  [[nodiscard]] auto closest(const Type& type) const -> Type {
    Type closest_sample;
    if (isSmaller(type)) {
      closest_sample = lowerBound();
    } else if (isGreater(type)) {
      closest_sample = upperBound();
    } else {
      closest_sample = type;
    }
    return closest_sample;
  };

  /// Checks whether two ranges intersect.
  /// \tparam OtherType Query type.
  /// \tparam other_boundary_policy Boundary policy of other range.
  /// \param other_range Other range.
  /// \return True if ranges intersect each other.
  template <typename OtherType, BoundaryPolicy other_boundary_policy>
  auto intersects(const Range<OtherType, other_boundary_policy>& other_range) const -> bool {
    return !(upperBound() < other_range.lowerBound() || other_range.upperBound() < lowerBound());
  }

  /// Computes the intersection of ranges.
  /// \tparam OtherType Other range type.
  /// \tparam other_boundary_policy Other boundary policy.
  /// \param other_range Other range.
  /// \return Pair containing boundary values.
  /// \note It is the user's responsibility to correctly handle the boundary policy!
  template <typename OtherType, BoundaryPolicy other_boundary_policy>
  auto intersection(const Range<OtherType, other_boundary_policy>& other_range) const
      -> std::pair<OtherType, OtherType> {
    const auto lower_value = lowerBound() < other_range.lowerBound() ? other_range.lowerBound() : lowerBound();
    const auto upper_value = upperBound() < other_range.upperBound() ? upperBound() : other_range.upperBound();
    return {lower_value, upper_value};
  }

  /// Stream operator.
  /// \param os Output stream.
  /// \param range Range to output.
  /// \return Modified output stream.
  friend auto operator<<(std::ostream& os, const Range& range) -> std::ostream& {
    if constexpr (boundary_policy == BoundaryPolicy::LOWER_INCLUSIVE_ONLY) {
      return os << "[" << range.lower << ", " << range.upper << ")";
    } else if constexpr (boundary_policy == BoundaryPolicy::UPPER_INCLUSIVE_ONLY) {
      return os << "(" << range.lower << ", " << range.upper << "]";
    } else if constexpr (boundary_policy == BoundaryPolicy::INCLUSIVE) {
      return os << "[" << range.lower << ", " << range.upper << "]";
    } else {
      return os << "(" << range.lower << ", " << range.upper << ")";
    }
  }

  Type lower;
  Type upper;
};

namespace internal {

template <typename Type>
struct BoundaryPolicyEvaluator {
  template <BoundaryPolicy boundary_policy>
  static auto LowerBound(const Range<Type, boundary_policy>& range) -> Type {
    if constexpr (boundary_policy == BoundaryPolicy::LOWER_INCLUSIVE_ONLY ||
                  boundary_policy == BoundaryPolicy::INCLUSIVE) {
      return range.lower;
    } else {
      return std::nextafter(range.lower, std::numeric_limits<Type>::max());
    }
  }

  template <BoundaryPolicy boundary_policy>
  static auto UpperBound(const Range<Type, boundary_policy>& range) -> Type {
    if constexpr (boundary_policy == BoundaryPolicy::UPPER_INCLUSIVE_ONLY ||
                  boundary_policy == BoundaryPolicy::INCLUSIVE) {
      return range.upper;
    } else {
      return std::nextafter(range.upper, -std::numeric_limits<Type>::max());
    }
  }

  template <BoundaryPolicy boundary_policy>
  static auto IsSmaller(const Range<Type, boundary_policy>& range, const Type& type) -> bool {
    return type < LowerBound(range);
  }

  template <BoundaryPolicy boundary_policy>
  static auto IsGreater(const Range<Type, boundary_policy>& range, const Type& type) -> bool {
    return UpperBound(range) < type;
  }

  template <BoundaryPolicy boundary_policy>
  static auto Span(const Range<Type, boundary_policy>& range) -> Type {
    return UpperBound(range) - LowerBound(range);
  }
};

template <class T>
  requires Real<T>
struct SamplingEvaluator;

template <Integral T>
struct SamplingEvaluator<T> {
  template <typename TGenerator>
  static auto Sample(const TGenerator& generator, const T& lower_bound, const T& span) -> T {
    std::uniform_real_distribution distribution(0, 1);
    return lower_bound + static_cast<T>(std::round(distribution(generator) * span()));
  }
};

template <Float T>
struct SamplingEvaluator<T> {
  template <typename TGenerator>
  static auto Sample(const TGenerator& generator, const T& lower_bound, const T& span) -> T {
    std::uniform_real_distribution distribution(0, 1);
    return lower_bound + static_cast<T>(distribution(generator) * span());
  }
};

}  // namespace internal

template <typename Type, BoundaryPolicy boundary_policy>
  requires Integral<Type> || Real<Type>
auto Range<Type, boundary_policy>::lowerBound() const -> Type {
  return internal::BoundaryPolicyEvaluator<Type>::LowerBound(*this);
}

template <typename Type, BoundaryPolicy boundary_policy>
  requires Integral<Type> || Real<Type>
auto Range<Type, boundary_policy>::upperBound() const -> Type {
  return internal::BoundaryPolicyEvaluator<Type>::UpperBound(*this);
}

template <typename Type, BoundaryPolicy boundary_policy>
  requires Integral<Type> || Real<Type>
auto Range<Type, boundary_policy>::isSmaller(const Type& type) const -> bool {
  return internal::BoundaryPolicyEvaluator<Type>::IsSmaller(*this, type);
}

template <typename Type, BoundaryPolicy boundary_policy>
  requires Integral<Type> || Real<Type>
auto Range<Type, boundary_policy>::isGreater(const Type& type) const -> bool {
  return internal::BoundaryPolicyEvaluator<Type>::IsGreater(*this, type);
}

template <typename Type, BoundaryPolicy boundary_policy>
  requires Integral<Type> || Real<Type>
auto Range<Type, boundary_policy>::span() const -> Type {
  return internal::BoundaryPolicyEvaluator<Type>::Span(*this);
}

template <typename Type, BoundaryPolicy boundary_policy>
  requires Integral<Type> || Real<Type>
template <typename TGenerator>
auto Range<Type, boundary_policy>::sample(const TGenerator& generator) const -> Type {
  return internal::SamplingEvaluator<Type>::Sample(generator);
}

}  // namespace hyperion
