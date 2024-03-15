// (c) 2021-2024 Philipp Ruppel

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <vector>

struct Face {
  std::array<size_t, 3> indices = {0, 0, 0};
  size_t material = 0;
  Face() {}
  Face(size_t a, size_t b, size_t c, size_t material)
      : indices({a, b, c}), material(material) {}
};

struct Edge {
  std::array<size_t, 2> indices;
  bool is_outline = false;
  Edge() {}
  Edge(size_t a, size_t b) : indices({a, b}) {}
};

struct Material {
  std::string name;
};

struct Mesh {
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Face> faces;
  std::vector<Edge> edges;
  std::vector<Material> materials;

  bool empty() const { return vertices.empty(); }

  Eigen::Vector3d computeCenter() const {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (auto &face : faces)
      for (size_t i : face.indices) center += vertices.at(i);
    center *= 1.0 / (faces.size() * 3);
    return center;
  }

  double computeSpread() const {
    Eigen::Vector3d center = computeCenter();
    double spread = 0.0;
    for (auto &face : faces) {
      for (size_t i : face.indices) {
        Eigen::Vector3d diff = vertices.at(i) - center;
        spread += diff.x() * diff.x();
        spread += diff.y() * diff.y();
        spread += diff.z() * diff.z();
      }
    }
    spread *= 1.0 / (faces.size() * 3 * 3);
    spread = std::sqrt(spread);
    return spread;
  }

  class Builder {
    std::vector<Eigen::Vector3d> _vertices;
    std::vector<Face> _faces;
    std::map<std::set<size_t>, size_t> _edges;
    std::map<std::array<double, 3>, size_t> _index_map;
    std::vector<Material> _materials;

    size_t _addVertex(const Eigen::Vector3d &vertex) {
      std::array<double, 3> v = {vertex.x(), vertex.y(), vertex.z()};
      if (_index_map.find(v) == _index_map.end()) {
        _index_map[v] = _vertices.size();
        _vertices.push_back(vertex);
      }
      return _index_map[v];
    }

   public:
    bool empty() const { return _vertices.empty(); }

    template <class It>
    void setMaterials(const It &begin, const It &end) {
      _materials.assign(begin, end);
    }

    void addTriangle(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                     const Eigen::Vector3d &c, size_t imaterial) {
      size_t ia = _addVertex(a);
      size_t ib = _addVertex(b);
      size_t ic = _addVertex(c);

      if (ia == ib || ib == ic || ic == ia) {
        std::cout << "WARNING: ignoring degenerate face" << std::endl;
        return;
      }

      _faces.emplace_back(ia, ib, ic, imaterial);

      _edges[{ia, ib}]++;
      _edges[{ib, ic}]++;
      _edges[{ic, ia}]++;
    }

    Mesh buildMesh() {
      Mesh mesh;
      mesh.vertices = _vertices;
      mesh.faces = _faces;
      mesh.materials = _materials;
      for (auto &e : _edges) {
        Edge edge;
        std::copy(e.first.begin(), e.first.end(), edge.indices.begin());
        edge.is_outline = (e.second == 1);
        mesh.edges.emplace_back(edge);
      }
      return mesh;
    }
  };

  std::vector<Mesh> split() const {
    std::vector<size_t> colors(vertices.size());
    std::iota(colors.begin(), colors.end(), 0);

    while (true) {
      std::cout << "loop" << std::endl;
      bool repeat = false;
      for (auto &face : faces) {
        std::array<size_t, 3> cc = {
            colors[face.indices[0]],
            colors[face.indices[1]],
            colors[face.indices[2]],
        };
        if (cc[0] != cc[1] || cc[1] != cc[2]) {
          repeat = true;
          size_t cmin = std::min(std::min(cc[0], cc[1]), cc[2]);
          for (size_t i = 0; i < 3; i++) {
            colors[face.indices[i]] = cmin;
          }
        }
      }
      if (!repeat) {
        break;
      }
    }

    for (auto &c : colors) {
      std::cout << c << " ";
    }
    std::cout << std::endl;

    std::vector<Mesh> ret;
    for (size_t color = 0; color < vertices.size(); color++) {
      Builder builder;
      builder.setMaterials(materials.begin(), materials.end());
      for (auto &face : faces) {
        if (color == colors[face.indices[0]]) {
          builder.addTriangle(vertices[face.indices[0]],
                              vertices[face.indices[1]],
                              vertices[face.indices[2]], face.material);
        }
      }
      if (!builder.empty()) {
        ret.push_back(builder.buildMesh());
      }
    }

    return ret;
  }
};

struct Mesh2d {
  std::vector<Face> faces;
  std::vector<Edge> edges;
  std::vector<Eigen::Vector2d> vertices;
  std::vector<Material> materials;

  Mesh toMesh() const {
    Mesh mesh;
    mesh.faces = faces;
    mesh.edges = edges;
    mesh.materials = materials;
    for (auto &v : vertices) mesh.vertices.emplace_back(v.x(), v.y(), 0.0);
    return mesh;
  }

  std::vector<Mesh2d> split() const {
    std::vector<Mesh2d> ret;
    for (auto &part : toMesh().split()) {
      Mesh2d part2d;
      part2d.faces = part.faces;
      part2d.edges = part.edges;
      part2d.materials = part.materials;
      for (auto &v : part.vertices) part2d.vertices.emplace_back(v.x(), v.y());
      ret.push_back(part2d);
    }
    return ret;
  }
};
