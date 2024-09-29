/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyperion/publishers/spline_control_point_publisher.hpp"
#include "hyperion/publishers/spline_knot_publisher.hpp"
#include "hyperion/publishers/spline_path_publisher.hpp"
#include "hyperion/publishers/spline_velocity_publisher.hpp"

namespace hyperion {

class SplinePublisher {
 public:
  using String = std::string;
  using Node = Shared<rclcpp::Node>;
  using QoS = rclcpp::QoS;

  SplinePublisher(const Node& node, const String& dir, const String& frame_id = "map", const QoS& qos = {10})
      : path_publisher_{node, frame_id, dir + "/path", qos},
        knots_publisher_{node, frame_id, dir + "/knots", qos},
        control_point_publisher_{node, frame_id, dir + "/control_points", qos},
        velocity_publisher_{node, frame_id, dir + "/velocity", qos} {}

  template <typename TSpline>
  auto publish(const TSpline& spline, const Duration& delta) -> void {
    path_publisher_.publish(spline, delta);
    knots_publisher_.publish(spline);
    control_point_publisher_.publish(spline);
    velocity_publisher_.publish(spline, delta);
  }

 private:
  SplinePathPublisher path_publisher_;
  SplineKnotPublisher knots_publisher_;
  SplineControlPointPublisher control_point_publisher_;
  SplineVelocityPublisher velocity_publisher_;
};

}  // namespace hyperion
