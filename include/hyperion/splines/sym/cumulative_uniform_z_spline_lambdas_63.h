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
 * Symbolic function: cumulative_uniform_z_spline_lambdas
 *
 * Args:
 *     ut: Scalar
 *
 * Outputs:
 *     res: Matrix54
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 5, 4> CumulativeUniformZSplineLambdas63(const Scalar ut) {
  // Total ops: 161

  // Input arrays

  // Intermediate terms (17)
  const Scalar _tmp0 = std::pow(ut, Scalar(5));
  const Scalar _tmp1 = -Scalar(5) / Scalar(24) * _tmp0;
  const Scalar _tmp2 = [&]() {
    const Scalar base = ut;
    return base * base * base;
  }();
  const Scalar _tmp3 = std::pow(ut, Scalar(4));
  const Scalar _tmp4 = (Scalar(1) / Scalar(12)) * ut;
  const Scalar _tmp5 = std::pow(ut, Scalar(2));
  const Scalar _tmp6 = (Scalar(1) / Scalar(24)) * _tmp5;
  const Scalar _tmp7 = (Scalar(5) / Scalar(6)) * _tmp0;
  const Scalar _tmp8 = -Scalar(7) / Scalar(12) * ut;
  const Scalar _tmp9 = (Scalar(5) / Scalar(8)) * _tmp5;
  const Scalar _tmp10 = -Scalar(25) / Scalar(24) * _tmp3;
  const Scalar _tmp11 = (Scalar(25) / Scalar(6)) * _tmp3;
  const Scalar _tmp12 = (Scalar(5) / Scalar(4)) * ut;
  const Scalar _tmp13 = -Scalar(25) / Scalar(6) * _tmp2;
  const Scalar _tmp14 = (Scalar(50) / Scalar(3)) * _tmp2;
  const Scalar _tmp15 = -Scalar(25) / Scalar(2) * _tmp5;
  const Scalar _tmp16 = 50 * _tmp5;

  // Output terms (1)
  Eigen::Matrix<Scalar, 5, 4> _res;

  _res(0, 0) = _tmp1 - Scalar(7) / Scalar(24) * _tmp2 + (Scalar(1) / Scalar(2)) * _tmp3 + 1;
  _res(1, 0) = (Scalar(13) / Scalar(12)) * _tmp2 - Scalar(49) / Scalar(24) * _tmp3 + _tmp4 + _tmp6 +
               _tmp7 + 1;
  _res(2, 0) = -Scalar(5) / Scalar(4) * _tmp0 - Scalar(5) / Scalar(3) * _tmp2 +
               (Scalar(25) / Scalar(8)) * _tmp3 + _tmp8 - _tmp9 + 1;
  _res(3, 0) =
      (Scalar(5) / Scalar(4)) * _tmp2 - Scalar(17) / Scalar(8) * _tmp3 + _tmp7 + _tmp8 + _tmp9;
  _res(4, 0) =
      _tmp1 - Scalar(3) / Scalar(8) * _tmp2 + (Scalar(13) / Scalar(24)) * _tmp3 + _tmp4 - _tmp6;
  _res(0, 1) = _tmp10 + 2 * _tmp2 - Scalar(7) / Scalar(8) * _tmp5;
  _res(1, 1) = _tmp11 - Scalar(49) / Scalar(6) * _tmp2 + _tmp4 + (Scalar(13) / Scalar(4)) * _tmp5 +
               Scalar(1) / Scalar(12);
  _res(2, 1) = -_tmp12 + (Scalar(25) / Scalar(2)) * _tmp2 - Scalar(25) / Scalar(4) * _tmp3 -
               5 * _tmp5 + Scalar(-7) / Scalar(12);
  _res(3, 1) = _tmp11 + _tmp12 - Scalar(17) / Scalar(2) * _tmp2 + (Scalar(15) / Scalar(4)) * _tmp5 +
               Scalar(-7) / Scalar(12);
  _res(4, 1) = _tmp10 + (Scalar(13) / Scalar(6)) * _tmp2 - _tmp4 - Scalar(9) / Scalar(8) * _tmp5 +
               Scalar(1) / Scalar(12);
  _res(0, 2) = _tmp13 + 6 * _tmp5 - Scalar(7) / Scalar(4) * ut;
  _res(1, 2) = _tmp14 - Scalar(49) / Scalar(2) * _tmp5 + (Scalar(13) / Scalar(2)) * ut +
               Scalar(1) / Scalar(12);
  _res(2, 2) = -25 * _tmp2 + (Scalar(75) / Scalar(2)) * _tmp5 - 10 * ut + Scalar(-5) / Scalar(4);
  _res(3, 2) = _tmp14 - Scalar(51) / Scalar(2) * _tmp5 + (Scalar(15) / Scalar(2)) * ut +
               Scalar(5) / Scalar(4);
  _res(4, 2) = _tmp13 + (Scalar(13) / Scalar(2)) * _tmp5 - Scalar(9) / Scalar(4) * ut +
               Scalar(-1) / Scalar(12);
  _res(0, 3) = _tmp15 + 12 * ut + Scalar(-7) / Scalar(4);
  _res(1, 3) = _tmp16 - 49 * ut + Scalar(13) / Scalar(2);
  _res(2, 3) = -75 * _tmp5 + 75 * ut - 10;
  _res(3, 3) = _tmp16 - 51 * ut + Scalar(15) / Scalar(2);
  _res(4, 3) = _tmp15 + 13 * ut + Scalar(-9) / Scalar(4);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym