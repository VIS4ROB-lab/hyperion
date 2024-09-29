/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <sym/pose3.h>

#include <rclcpp/node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hyperion/forward.hpp"
#include "hyperion/utils/clock.hpp"

namespace hyperion {

class SplineKnotPublisher {
 public:
  // Definitions.
  using String = std::string;
  using Node = Shared<rclcpp::Node>;

  using PointMsg = geometry_msgs::msg::Point;
  using KnotMsg = visualization_msgs::msg::Marker;
  using KnotPublisher = Shared<rclcpp::Publisher<KnotMsg>>;

  template <typename... Args>
  SplineKnotPublisher(Node node, const String& frame_id, Args&&... args)
      : node_{std::move(node)},
        publisher_{node_->create_publisher<KnotMsg>(std::forward<Args>(args)...)},
        frame_id_{frame_id} {}

  template <typename TScalar>
  static auto GetKnotPointMsg(const sym::Pose3<TScalar>& value) -> PointMsg {
    PointMsg msg;
    const auto& position = value.Position();
    msg.x = position.x();
    msg.y = position.y();
    msg.z = position.z();
    return msg;
  }

  template <typename TSpline>
  auto publish(const TSpline& spline) -> void {
    KnotMsg msg;
    msg.header.frame_id = frame_id_;
    const auto stamp = node_->now();
    msg.header.stamp = stamp;
    msg.ns = "knots";
    msg.id = 0;
    msg.type = KnotMsg::SPHERE_LIST;
    msg.action = KnotMsg::ADD;
    msg.pose.position.x = 0.0;
    msg.pose.position.z = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = 0.025;
    msg.scale.y = 0.025;
    msg.scale.z = 0.025;
    msg.color.a = 1.0;
    msg.color.r = 1.0;
    msg.color.g = 1.0;
    msg.color.b = 1.0;
    const auto num_poses = spline.controlPoints().size();
    msg.points.reserve(num_poses);
    for (Time t_i = spline.t0() + spline.dt(); t_i < spline.tn(); t_i += spline.dt()) {
      const auto value = spline.value(t_i);
      const auto point = GetKnotPointMsg(value);
      msg.points.emplace_back(point);
    }
    publisher_->publish(msg);
  }

 private:
  Node node_;
  KnotPublisher publisher_;
  String frame_id_;
};

}  // namespace hyperion
