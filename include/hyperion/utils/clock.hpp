/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <chrono>
#include <rclcpp/clock.hpp>

namespace hyperion {

struct Clock {
  // Definitions.
  using rep = std::int64_t;
  using period = std::nano;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<Clock, duration>;

  using Time = time_point;
  using Duration = duration;

  using RclCppTime = rclcpp::Time;
  using RclCppDuration = rclcpp::Duration;
  using RclCppClock = rclcpp::Clock;

  // Constants.
  static constexpr bool is_steady = false;

  /// Now in RclCpp time.
  /// \return RclCpp time.
  static auto RclCppNow() noexcept -> RclCppTime { return rclcpp_clock.now(); }

  /// Converts RclCpp to standard time.
  /// \param rcl_cpp_time RclCpp time.
  /// \return Standard time.
  static auto RclCppTimeToTime(const RclCppTime& rcl_cpp_time) -> Time {
    const auto nano_time = std::chrono::nanoseconds{rcl_cpp_time.nanoseconds()};
    return std::chrono::time_point<Clock>(nano_time);
  }

  /// Now in standard time.
  /// \return Standard time.
  static auto Now() noexcept -> Time { return RclCppTimeToTime(RclCppNow()); }

  /// Converts standard to RclCpp time.
  /// \param time Standard time.
  /// \return RclCpp time.
  static auto TimeToRclCppTime(const Time& time) -> RclCppTime {
    const auto nano_time = std::chrono::time_point_cast<std::chrono::nanoseconds>(time);
    return RclCppTime{nano_time.time_since_epoch().count()};
  }

  static RclCppClock rclcpp_clock;  ///< Clock.
};

using Time = Clock::Time;
using Duration = Clock::Duration;

}  // namespace hyperion
