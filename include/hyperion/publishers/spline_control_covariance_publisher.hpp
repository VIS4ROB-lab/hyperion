/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <rclcpp/node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "hyperion/forward.hpp"
#include "hyperion/utils/clock.hpp"

namespace hyperion {

class SplineControlCovariancePublisher {
 public:
  // Definitions.
  using String = std::string;
  using Node = Shared<rclcpp::Node>;

  using CovarianceMsg = visualization_msgs::msg::Marker;
  using CovarianceMsgs = visualization_msgs::msg::MarkerArray;
  using ControlCovariancePublisher = Shared<rclcpp::Publisher<CovarianceMsgs>>;

  template <typename... Args>
  SplineControlCovariancePublisher(Node node, const String& frame_id, Args&&... args)
      : node_{std::move(node)},
        publisher_{node_->create_publisher<CovarianceMsgs>(std::forward<Args>(args)...)},
        frame_id_{frame_id} {}

  template <typename TScalar>
  static auto GetCovarianceMsg(const sym::Pose3<TScalar>& value, const Eigen::Matrix<TScalar, 6, 6>& covariance)
      -> CovarianceMsg {
    // Definitions.
    using Matrix3 = Eigen::Matrix<TScalar, 3, 3>;

    // Compute the eigenvalues and eigenvectors.
    auto solver = Eigen::SelfAdjointEigenSolver<Matrix3>();
    solver.compute(covariance.template bottomRightCorner<3, 3>());
    const auto rot3 = sym::Rot3<TScalar>::FromRotationMatrix(solver.eigenvectors());

    // Convert to message.
    CovarianceMsg msg;
    msg.type = CovarianceMsg::SPHERE;
    msg.action = CovarianceMsg::ADD;
    msg.pose.position.x = value.Position().x();
    msg.pose.position.y = value.Position().y();
    msg.pose.position.z = value.Position().z();
    msg.pose.orientation.x = rot3.Data().x();
    msg.pose.orientation.y = rot3.Data().y();
    msg.pose.orientation.z = rot3.Data().z();
    msg.pose.orientation.w = rot3.Data().w();
    msg.scale.x = solver.eigenvalues().x();
    msg.scale.y = solver.eigenvalues().y();
    msg.scale.z = solver.eigenvalues().z();
    msg.color.a = 0.75;
    msg.color.r = 0.0;
    msg.color.g = 1.0;
    msg.color.b = 1.0;
    return msg;
  }

  template <typename TSpline>
  auto publish(const TSpline& spline) -> void {
    CovarianceMsgs msgs;
    const auto num_poses = spline.controlPoints().size();
    msgs.markers.reserve(num_poses);

    const auto& control_points = spline.controlPoints();
    const auto& control_covariances = spline.controlCovariances();

    const auto stamp = node_->now();
    for (std::size_t i = 0; i < num_poses; ++i) {
      auto covariance_msg = GetCovarianceMsg(control_points[i], control_covariances[i]);
      covariance_msg.header.frame_id = frame_id_;
      covariance_msg.header.stamp = stamp;
      covariance_msg.ns = "control_covariances";
      covariance_msg.id = i;
      msgs.markers.emplace_back(covariance_msg);
    }

    publisher_->publish(msgs);
  }

 private:
  Node node_;
  ControlCovariancePublisher publisher_;
  String frame_id_;
};

}  // namespace hyperion
