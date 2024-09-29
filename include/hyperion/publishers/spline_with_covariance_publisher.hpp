/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyperion/publishers/spline_control_covariance_publisher.hpp"
#include "hyperion/publishers/spline_publisher.hpp"

namespace hyperion {

class SplineWithCovariancePublisher : public SplinePublisher {
 public:
  SplineWithCovariancePublisher(const Node& node, const String& dir, const String& frame_id = "map",
                                const QoS& qos = {10})
      : SplinePublisher{node, dir, frame_id, qos},
        control_covariance_publisher_{node, frame_id, dir + "/control_covariances", qos} {}

  template <typename TSplineWithCovariance>
  auto publish(const TSplineWithCovariance& spline_with_covariance, const Duration& delta) -> void {
    SplinePublisher::publish(spline_with_covariance, delta);
    control_covariance_publisher_.publish(spline_with_covariance);
  }

 private:
  SplineControlCovariancePublisher control_covariance_publisher_;
};

}  // namespace hyperion
