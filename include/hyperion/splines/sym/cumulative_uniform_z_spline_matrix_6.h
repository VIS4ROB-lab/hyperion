// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: cumulative_uniform_z_spline_matrix
 *
 * Args:
 *
 * Outputs:
 *     res: Matrix56
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 5, 6> CumulativeUniformZSplineMatrix6() {
  // Total ops: 0

  // Input arrays

  // Intermediate terms (0)

  // Output terms (1)
  Eigen::Matrix<Scalar, 5, 6> _res;

  _res(0, 0) = 1;
  _res(1, 0) = 1;
  _res(2, 0) = 1;
  _res(3, 0) = 0;
  _res(4, 0) = 0;
  _res(0, 1) = 0;
  _res(1, 1) = Scalar(1) / Scalar(12);
  _res(2, 1) = Scalar(-7) / Scalar(12);
  _res(3, 1) = Scalar(-7) / Scalar(12);
  _res(4, 1) = Scalar(1) / Scalar(12);
  _res(0, 2) = 0;
  _res(1, 2) = Scalar(1) / Scalar(24);
  _res(2, 2) = Scalar(-5) / Scalar(8);
  _res(3, 2) = Scalar(5) / Scalar(8);
  _res(4, 2) = Scalar(-1) / Scalar(24);
  _res(0, 3) = Scalar(-7) / Scalar(24);
  _res(1, 3) = Scalar(13) / Scalar(12);
  _res(2, 3) = Scalar(-5) / Scalar(3);
  _res(3, 3) = Scalar(5) / Scalar(4);
  _res(4, 3) = Scalar(-3) / Scalar(8);
  _res(0, 4) = Scalar(1) / Scalar(2);
  _res(1, 4) = Scalar(-49) / Scalar(24);
  _res(2, 4) = Scalar(25) / Scalar(8);
  _res(3, 4) = Scalar(-17) / Scalar(8);
  _res(4, 4) = Scalar(13) / Scalar(24);
  _res(0, 5) = Scalar(-5) / Scalar(24);
  _res(1, 5) = Scalar(5) / Scalar(6);
  _res(2, 5) = Scalar(-5) / Scalar(4);
  _res(3, 5) = Scalar(5) / Scalar(6);
  _res(4, 5) = Scalar(-5) / Scalar(24);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym