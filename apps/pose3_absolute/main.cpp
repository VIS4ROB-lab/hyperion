/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "hyperion/hyperion.hpp"

using namespace hyperion;

struct Sphere {
  using Spline = splines::UniformZSplineWithCovariance<Pose3, 4>;

  static auto Create() -> Spline {
    const auto t0 = Clock::Time{std::chrono::milliseconds(0)};
    const auto dt = Clock::Duration{std::chrono::milliseconds(100)};
    const auto num_segemnts = 100;

    auto spline = Spline::Identity(t0, dt, num_segemnts);

    const auto radius = 1.0;
    const auto num_points = static_cast<int>(spline.controlPoints().size());
    for (auto i = 0; auto& control_point : spline.controlPoints()) {
      /* Scalar deltaTheta = 2 * M_PI / num_points;
      Scalar deltaPhi = M_PI / num_points;
      Scalar theta = i * deltaTheta;
      Scalar phi = std::acos(Scalar{1} - (Scalar{2} * i) / num_points); */

      R3 position;
      const auto z = 1.9 * (static_cast<Scalar>(i) / (num_points - 1) - 0.5);
      position.x() = radius * cos(sqrt(num_points * M_PI) * asin(z)) * sqrt(1 - z * z);
      position.y() = radius * sin(sqrt(num_points * M_PI) * asin(z)) * sqrt(1 - z * z);
      position.z() = radius * z;

      control_point = {Rot3::FromTangent(static_cast<Scalar>(i) / (num_points - 1) * R3::Ones()), position};
      ++i;
    }

    return spline;
  }
};

auto main(const int argc, char* argv[]) -> int {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("node");
  auto reference_publisher = SplineWithCovariancePublisher{node, "hyperion/spline/reference"};
  auto hyperion_publisher = SplineWithCovariancePublisher{node, "hyperion/spline/optimized"};
  auto ceres_publisher = SplineWithCovariancePublisher{node, "ceres/spline/optimized"};

  // Create reference spline.
  const auto reference_spline = Sphere::Create();

  // Create perturbed spline.
  const auto perturbation = 1.0;
  auto hyperion_spline = reference_spline;
  auto ceres_spline = reference_spline;
  for (std::size_t i = 0; i < reference_spline.controlPoints().size(); ++i) {
    auto& reference_cp = reference_spline.controlPoints()[i];
    const Rot3 perturbed_rotation = reference_cp.Rotation().Compose(Rot3::FromTangent(perturbation * R3::Random()));
    const R3 pertubed_position = reference_cp.Position() + perturbation * R3::Random();
    hyperion_spline.controlPoints()[i] = {perturbed_rotation, pertubed_position};
    ceres_spline.controlPoints()[i] = {perturbed_rotation, pertubed_position};
  }

  // Crete graph.
  GraphOptions graph_options{
      .cost_ownership = Ownership::TAKE_OWNERSHIP,
      .loss_ownership = Ownership::DO_NOT_TAKE_OWNERSHIP};
  Graph graph{graph_options};

  // Add values and covariances to graph.
  for (std::size_t i = 0; i < hyperion_spline.controlPoints().size(); ++i) {
    const auto value_ptr = const_cast<Scalar*>(hyperion_spline.controlPoints()[i].Data().data());
    const auto covariance_ptr = hyperion_spline.controlCovariances()[i].data();
    graph.addValue<Pose3>(ValueType::GAUSSIAN, value_ptr, covariance_ptr);
  }

  // Create Ceres problem.
  ::ceres::Problem problem;
  for (const auto & i : ceres_spline.controlPoints()) {
    const auto value_ptr = const_cast<Scalar*>(i.Data().data());
    //const auto covariance_ptr = hyperion_spline.controlCovariances()[i].data();
    problem.AddParameterBlock(value_ptr, Pose3::StorageDim(), new hyperion::ceres::ManifoldPose3{});
  }

  const auto noise = 0.05;
  const auto sqrt_info_measurement = 2.0;
  const auto dt = std::chrono::milliseconds(25);
  for (auto ti = hyperion_spline.t0(); ti < hyperion_spline.tn(); ti += dt) {
    // Hyperion.
    const auto value = reference_spline.value(ti);
    const Rot3 noisy_rotation = value.Rotation().Compose(Rot3::FromTangent(noise * R3::Random()));
    const R3 noisy_position = value.Position() + noise * R3::Random();
    const auto prior = Pose3{noisy_rotation, noisy_position};
    const auto ut = hyperion_spline.getNormalizedTime(ti);
    const auto ut_lambdas = sym::CumulativeUniformZSplineLambdas40(ut);
    auto cost = new Spline3Pose3PriorCostFunction{ut_lambdas, prior, sqrt_info_measurement * Matrix<6>::Identity()};
    const auto parameter_blocks = hyperion_spline.getParameterBlocks(ti);
    const auto ptrs = std::vector<const Scalar*>(std::begin(parameter_blocks), std::end(parameter_blocks));
    graph.addFactor(cost, nullptr, ptrs);

    // Ceres
    auto ceres_cost = new hyperion::ceres::Spline3Pose3PriorCostFunction{
        ut_lambdas,
        prior,
        sqrt_info_measurement * Matrix<6>::Identity()};
    const auto ceres_parameter_blocks = ceres_spline.getParameterBlocks(ti);
    const auto ceres_parameter_block_ptrs =
        std::vector<Scalar*>(std::begin(ceres_parameter_blocks), std::end(ceres_parameter_blocks));
    problem.AddResidualBlock(ceres_cost, nullptr, ceres_parameter_block_ptrs);
  }

  ::ceres::Solver::Options ceres_options;
  ::ceres::Solver::Summary ceres_summary;
  ceres_options.minimizer_progress_to_stdout = true;
  ceres_options.logging_type = ::ceres::LoggingType::PER_MINIMIZER_ITERATION;
  ::ceres::Solve(ceres_options, &problem, &ceres_summary);
  LOG(INFO) << ceres_summary.FullReport();

  SolverOptions solver_options;
  const auto summary = Solver::Solve(graph, SolverOptions{});

  /* std::ofstream hyperion__absolute_energy_file;
  hyperion__absolute_energy_file.open("hyperion_absolute_energy.csv");
  hyperion__absolute_energy_file << "iter, cost\n";
  hyperion__absolute_energy_file << 0 << ", " << E0 << "\n";
  hyperion__absolute_energy_file.close(); */

  const auto dt_csv = std::chrono::milliseconds(200);
  std::ofstream hyperion_file;
  hyperion_file.open("hyperion_absolute_rmse.csv");
  hyperion_file << "t, dr, dt\n";

  for (auto ti = hyperion_spline.t0(); ti < hyperion_spline.tn(); ti += dt_csv) {
    const auto ref_val = reference_spline.value(ti);
    const auto opt_val = hyperion_spline.value(ti);
    const auto delta = opt_val.LocalCoordinates(ref_val);
    const auto t = std::chrono::duration<Scalar>(ti.time_since_epoch()).count();
    hyperion_file << t << ", " << delta.head<3>().norm() << ", " << delta.tail<3>().norm() << "\n";
  }

  hyperion_file.close();

  std::ofstream ceres_file;
  ceres_file.open("ceres_absolute_rmse.csv");
  ceres_file << "t, dr, dt\n";

  for (auto ti = ceres_spline.t0(); ti < ceres_spline.tn(); ti += dt_csv) {
    const auto ref_val = reference_spline.value(ti);
    const auto opt_val = ceres_spline.value(ti);
    const auto delta = opt_val.LocalCoordinates(ref_val);
    const auto t = std::chrono::duration<Scalar>(ti.time_since_epoch()).count();
    ceres_file << t << ", " << delta.head<3>().norm() << ", " << delta.tail<3>().norm() << "\n";
  }

  ceres_file.close();

  const auto rmse_dt = std::chrono::milliseconds(200);
  const auto ceres_rmse = splines::Comparator::ComputeRMSE(ceres_spline, reference_spline, rmse_dt);
  const auto hyperion_rmse = splines::Comparator::ComputeRMSE(hyperion_spline, reference_spline, rmse_dt);
  LOG(INFO) << "Ceres RMSE: R: " << ceres_rmse.rotation_error << " t: " << ceres_rmse.position_error;
  LOG(INFO) << "Hyperion RMSE: R: " << hyperion_rmse.rotation_error << " t: " << hyperion_rmse.position_error;

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
