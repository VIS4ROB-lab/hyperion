/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <sym/pose3.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/node.hpp>

#include "hyperion/forward.hpp"
#include "hyperion/utils/clock.hpp"

namespace hyperion {

class SplinePosePublisher {
 public:
  // Definitions.
  using String = std::string;
  using Node = Shared<rclcpp::Node>;

  using PoseMsg = geometry_msgs::msg::Pose;
  using StampedPoseMsg = geometry_msgs::msg::PoseStamped;
  using PoseMsgs = geometry_msgs::msg::PoseArray;
  using PosePublisher = Shared<rclcpp::Publisher<PoseMsgs>>;

  template <typename... Args>
  SplinePosePublisher(Node node, const String& frame_id, Args&&... args)
      : node_{std::move(node)},
        publisher_{node_->create_publisher<PoseMsg>(std::forward<Args>(args)...)},
        frame_id_{frame_id} {}

  template <typename TScalar>
  static auto GetPoseMsg(const sym::Pose3<TScalar>& value) -> PoseMsg {
    PoseMsg msg;
    const auto& rotation = value.Rotation().Data();
    const auto& position = value.Position();
    msg.position.x = position.x();
    msg.position.y = position.y();
    msg.position.z = position.z();
    msg.orientation.x = rotation.x();
    msg.orientation.y = rotation.y();
    msg.orientation.z = rotation.z();
    msg.orientation.w = rotation.w();
    return msg;
  }

  template <typename TScalar>
  static auto GetStampedPoseMsg(const std_msgs::msg::Header& header, const sym::Pose3<TScalar>& value)
      -> StampedPoseMsg {
    StampedPoseMsg msg;
    msg.header = header;
    msg.pose = GetPoseMsg(value);
    return msg;
  }

  template <typename TSpline>
  auto publish(const TSpline& spline, const Duration& delta) -> void {
    PoseMsgs msg;
    msg.header.frame_id = frame_id_;
    const auto stamp = node_->now();
    msg.header.stamp = stamp;
    const auto num_msgs = spline.duration() / delta + 1;
    msg.poses.reserve(num_msgs);
    for (Time t_i = spline.t0(); t_i < spline.tn(); t_i += delta) {
      const auto value = spline.value(t_i);
      const auto pose = GetStampedPoseMsg(msg.header, value);
      msg.poses.emplace_back(pose);
    }
    publisher_->publish(msg);
  }

 protected:
  Node node_;
  PosePublisher publisher_;
  String frame_id_;
};

}  // namespace hyperion
