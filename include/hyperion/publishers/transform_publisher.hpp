/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <sym/pose3.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/node.hpp>

namespace hyperion {

class TransformPublisher {
 public:
  // Definitions.
  using String = std::string;
  using Node = Shared<rclcpp::Node>;

  using TransformMsg = geometry_msgs::msg::TransformStamped;
  using TransformBroadcaster = Shared<tf2_ros::TransformBroadcaster>;
  using QoS = rclcpp::QoS;

  TransformPublisher(Node node, const String& parent_frame_id, const String& child_frame_id, const QoS& qos = {10})
      : node_{node},
        transform_broadcaster_{std::make_shared<tf2_ros::TransformBroadcaster>(node, qos)},
        parent_frame_id_{parent_frame_id},
        child_frame_id_{child_frame_id} {}

  template <typename TScalar>
  auto publish(const sym::Pose3<TScalar>& value) -> void {
    TransformMsg msg;
    msg.header.frame_id = parent_frame_id_;
    msg.header.stamp = node_->now();
    msg.child_frame_id = child_frame_id_;

    auto& transform = msg.transform;
    const auto& rotation = value.Rotation().Data();
    const auto& position = value.Position();
    transform.rotation.x = rotation.x();
    transform.rotation.y = rotation.y();
    transform.rotation.z = rotation.z();
    transform.rotation.w = rotation.w();
    transform.translation.x = position.x();
    transform.translation.y = position.y();
    transform.translation.z = position.z();
    transform_broadcaster_->sendTransform(msg);
  }

 private:
  Node node_;
  TransformBroadcaster transform_broadcaster_;
  String parent_frame_id_;
  String child_frame_id_;
};

}  // namespace hyperion
