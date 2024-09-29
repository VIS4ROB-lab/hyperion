/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <sym/pose3.h>

#include <rclcpp/node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hyperion/forward.hpp"
#include "hyperion/utils/clock.hpp"

namespace hyperion {

class SplineVelocityPublisher {
 public:
  // Definitions.
  using String = std::string;
  using Node = Shared<rclcpp::Node>;

  using PointMsg = geometry_msgs::msg::Point;
  using VelocityMsg = visualization_msgs::msg::Marker;
  using VelocityMsgs = visualization_msgs::msg::MarkerArray;
  using VelocityPublisher = Shared<rclcpp::Publisher<VelocityMsgs>>;

  template <typename... Args>
  SplineVelocityPublisher(Node node, const String& frame_id, Args&&... args)
      : node_{std::move(node)},
        publisher_{node_->create_publisher<VelocityMsgs>(std::forward<Args>(args)...)},
        frame_id_{frame_id} {}

  template <typename TScalar>
  static auto GetVelocityMsg(const sym::Pose3<TScalar>& value, const Eigen::Vector<TScalar, 6>& velocity)
      -> std::pair<PointMsg, PointMsg> {
    PointMsg start_msg, end_msg;
    const auto& position = value.Position();
    start_msg.x = position.x();
    start_msg.y = position.y();
    start_msg.z = position.z();
    end_msg.x = position.x() + velocity[3];
    end_msg.y = position.y() + velocity[4];
    end_msg.z = position.z() + velocity[5];
    return {start_msg, end_msg};
  }

  template <typename TSpline>
  auto publish(const TSpline& spline, const Duration& delta) -> void {
    VelocityMsgs msgs;
    const auto num_msgs = spline.duration() / delta + 1;
    msgs.markers.reserve(num_msgs);

    int i = 0;
    const auto stamp = node_->now();
    for (Time t_i = spline.t0(); t_i < spline.tn(); t_i += delta) {
      VelocityMsg msg;
      msg.header.frame_id = frame_id_;
      msg.header.stamp = stamp;
      msg.ns = "velocity";
      msg.id = i;
      msg.type = VelocityMsg::ARROW;
      msg.action = VelocityMsg::ADD;
      const auto value = spline.value(t_i);
      const auto velocity = spline.velocity(t_i, true);
      msg.points.resize(2);
      std::tie(msg.points[0], msg.points[1]) = GetVelocityMsg(value, velocity);
      msg.pose.position.x = 0.0;
      msg.pose.position.z = 0.0;
      msg.pose.position.y = 0.0;
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0;
      msg.pose.orientation.w = 1.0;
      msg.scale.x = 0.005;
      msg.scale.y = 0.02;
      msg.scale.z = 0.01;
      msg.color.a = 1.0;
      msg.color.r = 1.0;
      msg.color.g = 1.0;
      msg.color.b = 0.0;
      msgs.markers.emplace_back(msg);
      ++i;
    }
    publisher_->publish(msgs);
  }

 protected:
  Node node_;
  VelocityPublisher publisher_;
  String frame_id_;
};

}  // namespace hyperion
