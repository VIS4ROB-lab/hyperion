// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include <sym/rot3.h>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: storage_d_tangent_rot3
 *
 * Args:
 *     value: Rot3
 *
 * Outputs:
 *     res: Matrix43
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 4, 3> StorageDTangentRot3(const sym::Rot3<Scalar>& value) {
  // Total ops: 7

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _value = value.Data();

  // Intermediate terms (7)
  const Scalar _tmp0 = (Scalar(1) / Scalar(2)) * _value[3];
  const Scalar _tmp1 = (Scalar(1) / Scalar(2)) * _value[2];
  const Scalar _tmp2 = (Scalar(1) / Scalar(2)) * _value[1];
  const Scalar _tmp3 = -_tmp2;
  const Scalar _tmp4 = (Scalar(1) / Scalar(2)) * _value[0];
  const Scalar _tmp5 = -_tmp4;
  const Scalar _tmp6 = -_tmp1;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 3> _res;

  _res(0, 0) = _tmp0;
  _res(1, 0) = _tmp1;
  _res(2, 0) = _tmp3;
  _res(3, 0) = _tmp5;
  _res(0, 1) = _tmp6;
  _res(1, 1) = _tmp0;
  _res(2, 1) = _tmp4;
  _res(3, 1) = _tmp3;
  _res(0, 2) = _tmp2;
  _res(1, 2) = _tmp5;
  _res(2, 2) = _tmp0;
  _res(3, 2) = _tmp6;

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
