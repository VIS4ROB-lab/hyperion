// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     FACTOR.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

namespace sym_hyperion {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: r3_between_factor
 *
 * Args:
 *     x: Matrix31
 *     x_T_y: Matrix31
 *     y: Matrix31
 *     sqrt_info: Matrix33
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix31
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> R3BetweenFactor(const Eigen::Matrix<Scalar, 3, 1>& x,
                                            const Eigen::Matrix<Scalar, 3, 1>& x_T_y,
                                            const Eigen::Matrix<Scalar, 3, 1>& y,
                                            const Eigen::Matrix<Scalar, 3, 3>& sqrt_info,
                                            const Scalar epsilon) {
  // Total ops: 21

  // Unused inputs
  (void)epsilon;

  // Input arrays

  // Intermediate terms (3)
  const Scalar _tmp0 = x(1, 0) - x_T_y(1, 0) - y(1, 0);
  const Scalar _tmp1 = x(2, 0) - x_T_y(2, 0) - y(2, 0);
  const Scalar _tmp2 = x(0, 0) - x_T_y(0, 0) - y(0, 0);

  // Output terms (1)
  Eigen::Matrix<Scalar, 3, 1> _res;

  _res(0, 0) = _tmp0 * sqrt_info(0, 1) + _tmp1 * sqrt_info(0, 2) + _tmp2 * sqrt_info(0, 0);
  _res(1, 0) = _tmp0 * sqrt_info(1, 1) + _tmp1 * sqrt_info(1, 2) + _tmp2 * sqrt_info(1, 0);
  _res(2, 0) = _tmp0 * sqrt_info(2, 1) + _tmp1 * sqrt_info(2, 2) + _tmp2 * sqrt_info(2, 0);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_hyperion