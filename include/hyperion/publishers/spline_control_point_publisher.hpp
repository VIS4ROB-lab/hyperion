/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyperion/publishers/spline_pose_publisher.hpp"

namespace hyperion {

class SplineControlPointPublisher {
 public:
  // Definitions.
  using String = std::string;
  using Node = Shared<rclcpp::Node>;

  using ControlPointMsgs = geometry_msgs::msg::PoseArray;
  using ControlPointPublisher = Shared<rclcpp::Publisher<ControlPointMsgs>>;

  template <typename... Args>
  SplineControlPointPublisher(Node node, const String& frame_id, Args&&... args)
      : node_{std::move(node)},
        publisher_{node_->create_publisher<ControlPointMsgs>(std::forward<Args>(args)...)},
        frame_id_{frame_id} {}

  template <typename TSpline>
  auto publish(const TSpline& spline) -> void {
    ControlPointMsgs msgs;
    msgs.header.frame_id = frame_id_;
    msgs.header.stamp = node_->now();
    const auto num_poses = spline.controlPoints().size();
    msgs.poses.reserve(num_poses);
    for (const auto& value : spline.controlPoints()) {
      msgs.poses.emplace_back(SplinePosePublisher::GetPoseMsg(value));
    }
    publisher_->publish(msgs);
  }

 protected:
  Node node_;
  ControlPointPublisher publisher_;
  String frame_id_;
};

}  // namespace hyperion
