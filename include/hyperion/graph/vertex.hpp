/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <mutex>

#include "hyperion/graph/forward.hpp"

namespace hyperion {

class Vertex {
 public:
  // Definitions.
  using Mutex = std::mutex;
  using Lock = std::unique_lock<Mutex>;

  /// Default destructor.
  virtual ~Vertex();

  /// Locks the vertex.
  /// @return Vertex lock.
  auto lock() const -> Lock;

  /// Tries to lock the vertex.
  /// @return Acquisition flag and vertex lock.
  auto tryLock() const -> std::pair<bool, Lock>;

  /// Retrieves the vertex type.
  /// @return Vertex type.
  [[nodiscard]] auto getVertexType() const -> VertexType;

 protected:
  /// Constructor from vertex type.
  /// @param vertex_type Vertex type.
  explicit Vertex(const VertexType& vertex_type);

 private:
  mutable Mutex mutex_;     ///< Mutex.
  VertexType vertex_type_;  ///< Vertex type.
};

}  // namespace hyperion
