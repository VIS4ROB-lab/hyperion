/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <random>
#include <type_traits>

namespace hyperion {

static auto& Generator() {
  static std::mt19937 generator(std::random_device{}());
  return generator;
}

template <typename T>
std::enable_if_t<std::is_floating_point_v<T>, T> standardUniformReal() {
  static std::uniform_real_distribution<T> distribution(0, 1);
  return distribution(Generator());
}

template <typename T>
std::enable_if_t<std::is_floating_point_v<T>, T> standardNormalReal() {
  static std::normal_distribution<T> distribution(0, 1);
  return distribution(Generator());
}

template <typename T>
std::enable_if_t<std::is_floating_point_v<T>, T> uniformReal(T min, T max) {
  std::uniform_real_distribution<T> distribution(min, max);
  return distribution(Generator());
}

template <typename T>
std::enable_if_t<std::is_integral_v<T>, T> uniformInt(T min, T max) {
  std::uniform_int_distribution<T> distribution(min, max);
  return distribution(Generator());
}

template <typename T>
auto maybeTrue(const T probability) -> bool {
  CHECK(0 <= probability && probability <= 1);
  return standardUniformReal<T>() < probability;
}

template <typename T>
auto maybeFalse(const T probability) -> bool {
  return !maybeTrue(probability);
}

}  // namespace hyperion
