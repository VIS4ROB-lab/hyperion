/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <glog/logging.h>

#include <Eigen/Cholesky>
#include <Eigen/LU>

//#include <Eigen/SVD>

namespace hyperion {

template <typename TScalar>
struct InversionTraits;

template <>
struct InversionTraits<double> {
  static constexpr auto kSwitchSize = 16;
  static constexpr auto kAllowPseudoInversion = true;
  static constexpr auto kAllowSingularSystems = true;
};

/// \brief Inverts a positive semi-definite, symmetric matrix.
/// \param A Positive semi-definite, symmetric matrix.
/// \return Pseudoinverse of A.
template <int TSize, typename TDerived>
auto invertPSDSMatrix(const Eigen::MatrixBase<TDerived>& A) -> Eigen::Matrix<typename TDerived::Scalar, TSize, TSize> {
  // Definitions.
  using Matrix = Eigen::Matrix<typename TDerived::Scalar, TSize, TSize>;

  // Casts and sanity checks.
  const auto& A_derived = A.derived();
  DCHECK_EQ(A_derived.rows(), A_derived.cols());
  const auto size = A_derived.rows();

  // Invert non-singular matrix.
  if constexpr (TSize > 0 && TSize < 5) {
    if (size > 0 && size < 5) {
      bool invertible;
      Matrix pinv_A{size, size};
      A_derived.computeInverseWithCheck(pinv_A, invertible);
      if (invertible) {
        return pinv_A;
      }
    }
  } else {
    if (const auto llt = A_derived.template selfadjointView<Eigen::Lower>().llt();
        llt.info() != Eigen::NumericalIssue) {
      return llt.solve(Matrix::Identity(size, size));
    }
  }

  if constexpr (InversionTraits<Scalar>::kAllowPseudoInversion) {
    // Invert singular matrix (i.e. pseudo-inversion).
    if (size < InversionTraits<Scalar>::kSwitchSize) {
      const auto lu = A_derived.fullPivLu();
      return lu.solve(Matrix::Identity(size, size));
    } else {
      const auto cod = A_derived.completeOrthogonalDecomposition();
      //const auto svd = A_derived.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Matrix::Identity(size, size));
      //const auto svd = A_derived.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Matrix::Identity(size, size));
      return cod.solve(Matrix::Identity(size, size));
    }
  } else {
    LOG(FATAL) << "Matrix is singular and pseudo-inversion has been disabled.";
    return {};
  }
}

/// \brief Pseudo-solves a linear system Ax = b.
/// \param A Positive semi-definite, symmetric matrix.
/// \param b Right-hand side of linear system.
/// \return Least-squares solution to Ax = b.
template <int TSize, typename TDerived, typename TOtherDerived>
auto pseudoSolvePSDSMatrix(const Eigen::MatrixBase<TDerived>& A, const Eigen::MatrixBase<TOtherDerived>& b)
    -> Eigen::Vector<typename TDerived::Scalar, TSize> {
  // Casts and sanity checks.
  const auto& A_derived = A.derived();
  const auto& b_derived = b.derived();
  DCHECK_EQ(A_derived.rows(), A_derived.cols());

  // Solve non-singular matrix.
  if (const auto llt = A_derived.template selfadjointView<Eigen::Lower>().llt(); llt.info() != Eigen::NumericalIssue) {
    return llt.solve(b_derived);
  }

  if constexpr (InversionTraits<Scalar>::kAllowSingularSystems) {
    // Solve singular matrix (i.e. least-squares solution via pseudo-inversion).
    const auto size = A_derived.rows();
    if (size < InversionTraits<Scalar>::kSwitchSize) {
      const auto lu = A_derived.fullPivLu();
      return lu.solve(b_derived);
    } else {
      const auto cod = A_derived.completeOrthogonalDecomposition();
      //const auto svd = A_derived.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_derived);
      //const auto svd = A_derived.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_derived);
      return cod.solve(b_derived);
    }
  } else {
    LOG(FATAL) << "Matrix is singular and singular solutions have been disabled.";
    return {};
  }
}

/// \brief Solves the linear systems Ax = b and Ay = B.
/// \param A Positive semi-definite, symmetric matrix.
/// \param b Right-hand side of linear system Ax = b.
/// \param B Right-hand side of linear system Ay = B.
/// \return Solutions to Ax = b and Ay = B.
template <int TSize, typename TDerived, typename TOtherDerived, typename TThirdDerived>
auto solvePSDSMatrix(const Eigen::MatrixBase<TDerived>& A, const Eigen::MatrixBase<TOtherDerived>& b,
                     const Eigen::MatrixBase<TThirdDerived>& B)
    -> std::pair<Eigen::Vector<typename TDerived::Scalar, TSize>,
                 Eigen::Matrix<typename TDerived::Scalar, TSize, TSize>> {
  // Casts and sanity checks.
  const auto& A_derived = A.derived();
  const auto& b_derived = b.derived();
  const auto& B_derived = B.derived();
  DCHECK_EQ(A_derived.rows(), A_derived.cols());
  DCHECK_EQ(A_derived.cols(), b_derived.rows());
  DCHECK_EQ(A_derived.cols(), B_derived.rows());

  // Solve non-singular matrix.
  if (const auto llt = A_derived.template selfadjointView<Eigen::Lower>().llt(); llt.info() != Eigen::NumericalIssue) {
    return {llt.solve(b_derived), llt.solve(B_derived)};
  }

  if constexpr (InversionTraits<Scalar>::kAllowSingularSystems) {
    // Solve singular matrix (i.e. least-squares solution via pseudo-inversion).
    const auto size = A_derived.rows();
    if (size < InversionTraits<Scalar>::kSwitchSize) {
      const auto lu = A_derived.fullPivLu();
      return {lu.solve(b_derived), lu.solve(B_derived)};
    } else {
      const auto cod = A_derived.completeOrthogonalDecomposition();
      //const auto svd = A_derived.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_derived);
      //const auto svd = A_derived.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_derived);
      return {cod.solve(b_derived), cod.solve(B_derived)};
    }
  } else {
    LOG(FATAL) << "Matrix is singular and singular solutions have been disabled.";
    return {};
  }
}

}  // namespace hyperion
