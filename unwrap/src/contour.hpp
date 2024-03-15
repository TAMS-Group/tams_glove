// (c) 2021-2024 Philipp Ruppel

#pragma once

#include "mesh.hpp"

std::vector<Eigen::Vector2d> findContour(
    const std::vector<Eigen::Vector2d> &vertices,
    const std::vector<Edge> &edges) {
  std::vector<Eigen::Vector2d> ret;

  std::map<size_t, std::set<size_t>> neighbors;
  for (auto &edge : edges) {
    if (edge.is_outline) {
      if (edge.indices[0] != edge.indices[1]) {
        neighbors[edge.indices[0]].insert(edge.indices[1]);
        neighbors[edge.indices[1]].insert(edge.indices[0]);
      }
    }
  }

  if (neighbors.begin() != neighbors.end()) {
    size_t current = neighbors.begin()->first;
    for (size_t i = 0;; i++) {
      Eigen::Vector2d p = vertices[current];
      ret.push_back(p);

      size_t next = current;
      for (auto &n : neighbors[current]) {
        next = n;
        break;
      }
      neighbors[current].erase(next);
      neighbors[next].erase(current);

      if (next == current) {
        break;
      }

      current = next;
    }
  }

  return ret;
}
