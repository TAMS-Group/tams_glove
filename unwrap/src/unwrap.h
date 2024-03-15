// (c) 2021-2024 Philipp Ruppel

#pragma once

#include "mesh.h"

#include <functional>

#include <Eigen/Sparse>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

struct Unwrapper {
  Mesh _mesh;
  std::vector<Eigen::Vector3d> _vertices;
  std::vector<Face> _faces;
  std::vector<Eigen::Vector2d> _uv;
  std::vector<Eigen::Matrix<double, 3, 2>> _faces2d;

  double _temperature = 1;
  size_t _iterations = 10000;

  struct Status {
    size_t current_iteration = 0;
    size_t iteration_count = 0;
    double loss = 0.0;
    std::vector<Eigen::Vector2d> uv;
  };

  std::function<void(const Status &)> _callback;
  template <class Callback>
  void callback(const Callback &callback) {
    _callback = callback;
  };

  static void visualize(const Mesh &mesh,
                        const std::vector<Eigen::Vector2d> &solution) {
    cv::Mat_<cv::Vec3b> image(460 * 2, 640 * 2, cv::Vec3b(0, 0, 0));
    size_t s = (image.cols + image.rows) / 400;
    auto vertex = [&](size_t i) {
      return cv::Point(solution[i].x() * s + image.cols / 2,
                       solution[i].y() * s + image.rows / 2);
    };
    for (auto &edge : mesh.edges) {
      cv::line(image, vertex(edge.indices[0]), vertex(edge.indices[1]),
               cv::Scalar(255, 255, 255));
    }
    cv::imshow("preview", image);
    int key = cv::waitKey(1);
    std::cout << "key " << key << std::endl;
    if (key == 113 || key == 27) {
      exit(0);
    }
  }

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

  void solveGaussNewtonStep(double overrelaxation) {
    std::vector<Eigen::Triplet<double>> gradients;
    std::vector<double> residuals;

    for (size_t iface = 0; iface < _faces.size(); iface++) {
      auto &face = _faces[iface];
      auto projection = computeFaceProjection(iface);
      size_t i0 = residuals.size();
      for (size_t edge = 0; edge < 3; edge++) {
        for (size_t dim = 0; dim < 2; dim++) {
          double weight = 1;
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

    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower> solver;
    solver.setMaxIterations(500);
    solver.compute(gradient_matrix_2);
    solution = solver.solveWithGuess(residual_vector_2, solution);

    for (size_t i = 0; i < _uv.size(); i++) {
      _uv[i] += (solution.segment(i * 2, 2) - _uv[i]) * overrelaxation;
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

  void solveVisual() {
    auto prev_uv = _uv;

    auto toMatrix = [&](const std::vector<Eigen::Vector2d> &data) {
      Eigen::MatrixXd mat(2, data.size());
      for (size_t i = 0; i < data.size(); i++) {
        mat.col(i) = data[i];
      }
      return mat;
    };

    auto computeStepSize = [&]() {
      Eigen::Matrix3d tf =
          Eigen::umeyama(toMatrix(prev_uv), toMatrix(_uv), false);
      double step_size_sq = 0;
      for (size_t i = 0; i < _uv.size(); i++) {
        Eigen::Vector2d prev = (tf * prev_uv[i].homogeneous()).head(2);
        Eigen::Vector2d diff = prev - _uv[i];
        step_size_sq += diff.squaredNorm();
      }
      return std::sqrt(step_size_sq);
    };

    size_t n = _iterations;
    std::vector<Eigen::Vector2d> _velocities;
    _velocities.resize(_uv.size(), Eigen::Vector2d::Zero());
    Status status;
    status.iteration_count = n;
    for (size_t i = 0; i < n; i++) {
      double temperature = 1.0 - (i * 1.0 / (n - 1));
      temperature *= temperature * temperature;
      prev_uv = _uv;
      std::cout << "iteration " << i << " / " << n << " temperature "
                << temperature << std::endl;
      for (size_t i = 0; i < _uv.size(); i++) {
        _uv[i] += Eigen::Vector2d::Random() * temperature * _temperature;
      }
      solveGaussNewtonStep(1);
      if (_callback) {
        double cost = 0;
        for (auto &face : _faces) {
          for (size_t i = 0; i < 3; i++) {
            size_t j = (i + 1) % 3;
            double a = (_uv[face.indices[i]] - _uv[face.indices[j]]).norm();
            double b = (_vertices[face.indices[i]] - _vertices[face.indices[j]])
                           .norm();
            double d = a - b;
            cost += d * d;
          }
        }
        status.current_iteration = i;
        status.loss = cost;
        status.uv = _uv;
        _callback(status);
      }
      if (i % 64 == 0) {
        visualize(_mesh, _uv);
      }
      for (size_t i = 0; i < _uv.size(); i++) {
        _velocities[i] *= 0.9;
        _velocities[i] += (_uv[i] - prev_uv[i]) * 0.1;
      }
      for (size_t i = 0; i < _uv.size(); i++) {
        _uv[i] += _velocities[i];
      }
    }
  }

  void init(const Mesh &mesh) {
    _mesh = mesh;

    _vertices = mesh.vertices;
    _faces = mesh.faces;

    _uv.resize(_vertices.size());
    for (auto &v : _uv) {
      v.setRandom();
      v *= 100;
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
