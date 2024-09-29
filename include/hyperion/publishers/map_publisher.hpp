/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <Eigen/Core>
#include <rclcpp/node.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace hyperion {

class MapPublisher {
 public:
  // Definitions.
  using String = std::string;

  template <typename T>
  using Shared = std::shared_ptr<T>;

  using PointMsg = geometry_msgs::msg::Point;
  using MarkerMsg = visualization_msgs::msg::Marker;

  using SharedNode = Shared<rclcpp::Node>;
  using SharedMarkerPublisher = Shared<rclcpp::Publisher<MarkerMsg>>;
  using QoS = rclcpp::QoS;

  static constexpr auto kDefaultFrameId = "map";

  MapPublisher(SharedNode shared_node, const String& topic, const String& frame_id = kDefaultFrameId,
               const QoS& qos = {10})
      : shared_node_{std::move(shared_node)},
        shared_landmark_publisher_{shared_node_->create_publisher<MarkerMsg>(topic + "/landmarks", qos)},
        frame_id_{frame_id} {}

  template <typename TScalar>
  auto publish(const std::vector<Eigen::Vector<TScalar, 3>>& landmarks) -> void {
    MarkerMsg msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = shared_node_->now();
    msg.ns = "landmarks";
    msg.id = 0;
    msg.type = MarkerMsg::POINTS;
    msg.action = MarkerMsg::ADD;
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
    msg.color.r = 0.0;
    msg.color.g = 1.0;
    msg.color.b = 0.0;

    msg.points.reserve(landmarks.size());
    for (const auto& landmark : landmarks) {
      PointMsg point_msg;
      point_msg.x = landmark.x();
      point_msg.y = landmark.y();
      point_msg.z = landmark.z();
      msg.points.emplace_back(point_msg);
    }

    shared_landmark_publisher_->publish(msg);
  }

 private:
  SharedNode shared_node_;                           ///< Node.
  SharedMarkerPublisher shared_landmark_publisher_;  ///< Landmark publisher.
  String frame_id_;                                  ///< Frame ID.
};

}  // namespace hyperion
