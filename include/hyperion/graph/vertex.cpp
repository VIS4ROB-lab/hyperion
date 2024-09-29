/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/graph/vertex.hpp"

namespace hyperion {

Vertex::~Vertex() = default;

auto Vertex::lock() const -> Lock {
  return Lock{mutex_};
}

auto Vertex::tryLock() const -> std::pair<bool, Lock> {
  auto lock = Lock{mutex_, std::defer_lock};
  return {lock.try_lock(), std::move(lock)};
}

auto Vertex::getVertexType() const -> VertexType {
  return vertex_type_;
}

Vertex::Vertex(const VertexType& vertex_type) : mutex_{}, vertex_type_{vertex_type} {}

}  // namespace hyperion
