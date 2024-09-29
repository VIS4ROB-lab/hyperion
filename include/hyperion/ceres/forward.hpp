/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <sym/util/epsilon.h>

#include <Eigen/Dense>
#include <memory>

namespace hyperion::ceres {

// Definitions.
using Scalar = double;

template <int TNumRows = Eigen::Dynamic, int TNumCols = TNumRows>
using Matrix = Eigen::Matrix<Scalar, TNumRows, TNumCols, TNumCols == 1 ? Eigen::ColMajor : Eigen::RowMajor>;

template <int TNumRows = Eigen::Dynamic, int TNumCols = TNumRows>
using MatrixMap = Eigen::Map<Matrix<TNumRows, TNumCols>>;

template <int TNumRows = Eigen::Dynamic, int TNumCols = TNumRows>
using ConstMatrixMap = Eigen::Map<const Matrix<TNumRows, TNumCols>>;

template <int TNumRows = Eigen::Dynamic, int TNumCols = TNumRows>
using MatrixRef = Eigen::Ref<Matrix<TNumRows, TNumCols>>;

template <int TNumRows = Eigen::Dynamic, int TNumCols = TNumRows>
using ConstMatrixRef = Eigen::Ref<const Matrix<TNumRows, TNumCols>>;

template <int TNumRows = Eigen::Dynamic>
using Vector = Eigen::Matrix<Scalar, TNumRows, 1>;

template <int TNumRows = Eigen::Dynamic>
using VectorMap = Eigen::Map<Vector<TNumRows>>;

template <int TNumRows = Eigen::Dynamic>
using ConstVectorMap = Eigen::Map<const Vector<TNumRows>>;

template <int TNumRows = Eigen::Dynamic>
using VectorRef = Eigen::Ref<Vector<TNumRows>>;

template <int TNumRows = Eigen::Dynamic>
using ConstVectorRef = Eigen::Ref<const Vector<TNumRows>>;

using Index = std::ptrdiff_t;

template <typename T>
using Unique = std::unique_ptr<T>;

template <typename T>
using Shared = std::shared_ptr<T>;

// Constants.
static constexpr auto kDefaultEpsilon = sym::kDefaultEpsilon<Scalar>;

template <int Dim>
class ManifoldRn;
class ManifoldRot2;
class ManifoldRot3;
class ManifoldPose2;
class ManifoldPose3;

}  // namespace hyperion::ceres
