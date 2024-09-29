/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <nav_msgs/msg/path.hpp>

#include "hyperion/publishers/spline_pose_publisher.hpp"

namespace hyperion {

class SplinePathPublisher {
 public:
  // Definitions.
  using String = std::string;
  using Node = Shared<rclcpp::Node>;

  using PathMsg = nav_msgs::msg::Path;
  using PathPublisher = Shared<rclcpp::Publisher<PathMsg>>;

  template <typename... Args>
  SplinePathPublisher(Node node, const String& frame_id, Args&&... args)
      : node_{std::move(node)},
        publisher_{node_->create_publisher<PathMsg>(std::forward<Args>(args)...)},
        frame_id_{frame_id} {}

  template <typename TSpline>
  auto publish(const TSpline& spline, const Duration& delta) -> void {
    PathMsg msg;
    msg.header.frame_id = frame_id_;
    const auto stamp = node_->now();
    msg.header.stamp = stamp;
    const auto num_msgs = spline.duration() / delta + 1;
    msg.poses.reserve(num_msgs);
    for (Time t_i = spline.t0(); t_i < spline.tn(); t_i += delta) {
      const auto value = spline.value(t_i);
      const auto pose = SplinePosePublisher::GetStampedPoseMsg(msg.header, value);
      msg.poses.emplace_back(pose);
    }
    publisher_->publish(msg);
  }

 private:
  Node node_;
  PathPublisher publisher_;
  String frame_id_;
};

}  // namespace hyperion
