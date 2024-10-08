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
 *     res: Matrix34
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 4> CumulativeUniformZSplineMatrix4() {
  // Total ops: 0

  // Input arrays

  // Intermediate terms (0)

  // Output terms (1)
  Eigen::Matrix<Scalar, 3, 4> _res;

  _res(0, 0) = 1;
  _res(1, 0) = 0;
  _res(2, 0) = 0;
  _res(0, 1) = Scalar(1) / Scalar(2);
  _res(1, 1) = Scalar(1) / Scalar(2);
  _res(2, 1) = 0;
  _res(0, 2) = -1;
  _res(1, 2) = Scalar(3) / Scalar(2);
  _res(2, 2) = Scalar(-1) / Scalar(2);
  _res(0, 3) = Scalar(1) / Scalar(2);
  _res(1, 3) = -1;
  _res(2, 3) = Scalar(1) / Scalar(2);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
