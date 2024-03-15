// (c) 2021-2024 Philipp Ruppel

#pragma once

#include "mesh.hpp"

#include <functional>

#include <Eigen/Sparse>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <random>

struct Unwrapper {
  Mesh _mesh;
  std::vector<Eigen::Vector3d> _vertices;
  std::vector<Face> _faces;
  std::vector<Eigen::Vector2d> _uv, _prev_uv;
  std::vector<Eigen::Matrix<double, 3, 2>> _faces2d;
  std::vector<Eigen::Vector2d> _velocities;

  double _temperature = 1;
  size_t _iterations = 20000;

  double _overrelaxation = 1.4;
  size_t _iteration = 0;
  double _spread = 0;

  Eigen::Matrix<double, 3, 2> computeFaceProjection(size_t iface) {
    auto &face = _faces.at(iface);

    Eigen::Matrix<double, 3, 2> tt = _faces2d.at(iface);

    Eigen::Matrix<double, 3, 2> uv;
    uv.row(0) = _uv[face.indices[0]];
    uv.row(1) = _uv[face.indices[1]];
    uv.row(2) = _uv[face.indices[2]];

    Eigen::Matrix3d tf = Eigen::umeyama(tt.transpose(), uv.transpose(), false);

    Eigen::Matrix<double, 3, 2> ret;

    for (size_t i = 0; i < 3; i++) {
      ret.row(i) = (tt.row(i).homogeneous() * tf.transpose()).head(2);
    }

    return ret;
  }

  void solveGaussNewtonStep() {
    std::vector<Eigen::Triplet<double>> gradients;
    std::vector<double> residuals;

    for (size_t iface = 0; iface < _faces.size(); iface++) {
      auto &face = _faces[iface];
      auto projection = computeFaceProjection(iface);
      size_t i0 = residuals.size();
      for (size_t edge = 0; edge < 3; edge++) {
        double weight = 1;
        for (size_t dim = 0; dim < 2; dim++) {
          gradients.emplace_back(i0 + edge * 2 + dim,
                                 face.indices[(edge + 1) % 3] * 2 + dim,
                                 +weight);
          gradients.emplace_back(i0 + edge * 2 + dim,
                                 face.indices[(edge + 2) % 3] * 2 + dim,
                                 -weight);
          residuals.push_back((projection((edge + 1) % 3, dim) -
                               projection((edge + 2) % 3, dim)) *
                              weight);
        }
      }
    }

    Eigen::SparseMatrix<double> gradient_matrix(residuals.size(),
                                                _uv.size() * 2);
    gradient_matrix.setFromTriplets(gradients.begin(), gradients.end());

    Eigen::VectorXd residual_vector(residuals.size());
    for (size_t i = 0; i < residuals.size(); i++) {
      residual_vector[i] = residuals[i];
    }

    Eigen::SparseMatrix<double> gradient_matrix_2 =
        gradient_matrix.transpose() * gradient_matrix;

    Eigen::VectorXd residual_vector_2 =
        gradient_matrix.transpose() * residual_vector;

    Eigen::VectorXd solution;
    solution.resize(_uv.size() * 2);
    for (size_t i = 0; i < _uv.size(); i++) {
      solution.segment(i * 2, 2) = _uv[i];
    }

    if (1) {
      Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower>
          solver;
      solver.setMaxIterations(500);
      solver.compute(gradient_matrix_2);
      solution = solver.solveWithGuess(residual_vector_2, solution);
    }

    if (0) {
      Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>
          solver;

      solver.analyzePattern(gradient_matrix_2);

      solver.factorize(gradient_matrix_2);

      solution = solver.solve(residual_vector_2);
    }

    for (size_t i = 0; i < _uv.size(); i++) {
      _uv[i] += (solution.segment(i * 2, 2) - _uv[i]) * _overrelaxation;
    }

    Eigen::Vector2d center;
    center.setZero();
    for (auto &uv : _uv) {
      center += uv;
    }
    center /= _uv.size();
    for (auto &uv : _uv) {
      uv -= center;
    }
  }

  void start() {
    _prev_uv = _uv;
    _velocities.resize(_uv.size(), Eigen::Vector2d::Zero());
    _iteration = 0;
  }

  cv::Mat visualize(size_t size) {
    std::cout << _iteration * 100.0 / _iterations << "%" << std::endl;

    cv::Mat_<cv::Vec3b> image(size, size, cv::Vec3b(0, 0, 0));

    double s = size / (_spread * 2 * 2);
    auto vertex = [&](size_t i) {
      return cv::Point(_uv[i].x() * s + image.cols / 2,
                       _uv[i].y() * s + image.rows / 2);
    };
    for (auto &edge : _mesh.edges) {
      cv::line(image, vertex(edge.indices[0]), vertex(edge.indices[1]),
               cv::Scalar(255, 255, 255));
    }

    return image;
  }

  void step() {
    double temperature = 1.0 - (_iteration * 1.0 / (_iterations - 1));
    temperature *= temperature * temperature;
    _prev_uv = _uv;

    for (size_t i = 0; i < _uv.size(); i++) {
      _uv[i] += Eigen::Vector2d::Random() * temperature * _temperature;
    }
    solveGaussNewtonStep();
    for (size_t i = 0; i < _uv.size(); i++) {
      _velocities[i] *= 0.9;
      _velocities[i] += (_uv[i] - _prev_uv[i]) * 0.1;
    }
    for (size_t i = 0; i < _uv.size(); i++) {
      _uv[i] += _velocities[i];
    }
    _iteration++;
  }

  bool ready() { return _iteration >= _iterations; }

  void rescale() {
    double sreal = 0.0;
    double scurr = 0.0;
    for (size_t iface = 0; iface < _faces.size(); iface++) {
      auto &face = _faces[iface];
      for (size_t edge = 0; edge < 3; edge++) {
        size_t vi = (edge + 0) % 3;
        size_t vj = (edge + 1) % 3;
        double lreal =
            (_vertices[face.indices[vi]] - _vertices[face.indices[vj]]).norm();
        double lcurr = (_uv[face.indices[vi]] - _uv[face.indices[vj]]).norm();
        sreal += lreal / lcurr;
        scurr += 1.0;
      }
    }
    double s = sreal / scurr;
    s *= s;
    std::cout << "rescaling " << s << std::endl;
    for (auto &v : _uv) {
      v *= s;
    }
  }

  void init(const Mesh &mesh) {
    _mesh = mesh;

    _vertices = mesh.vertices;
    _faces = mesh.faces;

    {
      _spread = mesh.computeSpread() * 3;
      std::mt19937 rng;
      std::normal_distribution<double> dist(0.0, _spread);
      _uv.resize(_vertices.size());
      for (auto &v : _uv) {
        v.x() = dist(rng);
        v.y() = dist(rng);
      }
    }

    _faces2d.resize(_faces.size());
    for (size_t iface = 0; iface < _faces.size(); iface++) {
      auto &face = _faces.at(iface);

      auto &p0 = _vertices[face.indices[0]];
      auto &p1 = _vertices[face.indices[1]];
      auto &p2 = _vertices[face.indices[2]];

      double l0 = (p1 - p2).norm();
      double l1 = (p0 - p2).norm();
      double l2 = (p0 - p1).norm();

      Eigen::Vector2d t0 = Eigen::Vector2d(0, 0);
      Eigen::Vector2d t1 = Eigen::Vector2d(l2, 0);

      double angle0 = std::acos((l1 * l1 + l2 * l2 - l0 * l0) / (2 * l1 * l2));
      Eigen::Vector2d t2 =
          Eigen::Vector2d(std::cos(angle0), std::sin(angle0)) * l1;

      Eigen::Matrix<double, 3, 2> tt;
      tt.row(0) = t0;
      tt.row(1) = t1;
      tt.row(2) = t2;

      _faces2d.at(iface) = tt;
    }
  }
};
