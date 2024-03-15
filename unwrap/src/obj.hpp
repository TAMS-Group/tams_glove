// (c) 2021-2024 Philipp Ruppel

#pragma once

#include "mesh.hpp"
#include <Eigen/Dense>
#include <array>
#include <fstream>
#include <iostream>

class OBJ {
 private:
  std::vector<Eigen::Vector3d> _vertices;
  std::map<std::set<size_t>, size_t> _edges;
  std::vector<Face> _faces;
  std::vector<Material> _materials;

  std::vector<std::string> _split(const std::string &str) {
    std::stringstream ss(str);
    std::vector<std::string> ret;
    for (std::string tok; ss >> tok;) {
      ret.push_back(tok);
    }
    return ret;
  }

  void _addFace(int ia, int ib, int ic, size_t imaterial) {
    if (ia < 0 || ib < 0 || ic < 0) {
      return;
    }

    size_t a = size_t(ia);
    size_t b = size_t(ib);
    size_t c = size_t(ic);

    _faces.emplace_back(a, b, c, imaterial);

    _edges[{a, b}]++;
    _edges[{b, c}]++;
    _edges[{a, c}]++;
  }

 public:
  void clear() {
    _vertices.clear();
    _edges.clear();
    _faces.clear();
    _materials.clear();
  }

  Mesh mesh() const {
    Mesh ret;
    ret.vertices = _vertices;
    ret.faces = _faces;
    ret.materials = _materials;
    for (auto &e : _edges) {
      Edge edge;
      std::copy(e.first.begin(), e.first.end(), edge.indices.begin());
      edge.is_outline = (e.second == 1);
      ret.edges.emplace_back(edge);
    }
    return ret;
  }

  OBJ() {}

  struct Options {
    std::function<bool(const std::string &)> object_filter;
    std::function<bool(const std::string &)> material_filter;
  };

  void load(const std::string &filename, const Options &opts = Options()) {
    std::ifstream stream(filename);

    clear();

    size_t imaterial = _materials.size();
    _materials.emplace_back();

    bool object_match = true;
    bool material_match = true;

    for (std::string line; getline(stream, line);) {
      auto toks = _split(line);

      if (toks[0] == "o") {
        object_match =
            (opts.object_filter ? opts.object_filter(toks[1]) : true);
      }

      if (toks[0] == "usemtl") {
        material_match =
            (opts.material_filter ? opts.material_filter(toks[1]) : true);
        if (material_match) {
          imaterial = _materials.size();
          Material mat;
          mat.name = toks[1];
          _materials.push_back(mat);
        }
      }

      if (toks[0] == "v" && toks.size() > 3) {
        double x = std::stod(toks[1]);
        double y = std::stod(toks[2]);
        double z = std::stod(toks[3]);

        _vertices.emplace_back(x, y, z);
      }

      if (toks[0] == "f" && toks.size() > 3 && object_match && material_match) {
        int a = std::stoi(toks[1]);
        int b = std::stoi(toks[2]);
        for (size_t i = 3; i < toks.size(); i++) {
          int c = std::stoi(toks[i]);
          _addFace(a - 1, b - 1, c - 1, imaterial);
          b = c;
        }
      }
    }
  }
};
