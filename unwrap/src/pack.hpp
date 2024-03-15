// (c) 2021-2024 Philipp Ruppel

#pragma once

#include "box.hpp"
#include "mesh.hpp"

#include <chrono>
#include <random>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

struct Packer {
  typedef Eigen::Affine2d Pose2d;
  typedef Eigen::Vector2d Vec2d;
  typedef std::array<double, 3> Cost;

  static constexpr double padding = 1;

  static constexpr double step_rotation = 1.0;
  static constexpr double step_translation = 3.0;

  struct Status {
    size_t current_iteration = 0;
    size_t iteration_count = 0;
    Cost cost;
    std::vector<Pose2d> poses;
    double temperature = 0.0;
  };

  std::function<void(const Status &)> _callback;
  template <class Callback>
  void callback(const Callback &callback) {
    _callback = callback;
  };

  static void transform(const Pose2d &pose, const Mesh2d &mesh,
                        std::vector<Vec2d> &buffer) {
    auto &vertices = mesh.vertices;
    for (size_t i = 0; i < vertices.size(); i++) {
      buffer[i] = pose * vertices[i];
    }
  }

  static void transform(const std::vector<Pose2d> &poses,
                        const std::vector<Mesh2d> &parts,
                        std::vector<std::vector<Vec2d>> &buffers) {
    for (size_t i = 0; i < poses.size(); i++) {
      transform(poses[i], parts[i], buffers[i]);
    }
  }

  static Box bounds(const std::vector<Vec2d> &points) {
    Box ret = Box::Empty();
    for (auto &p : points) {
      ret.add(p);
    }
    return ret;
  }

  static Box bounds(const std::vector<std::vector<Vec2d>> &points) {
    Box ret = Box::Empty();
    for (auto &pp : points) {
      for (auto &p : pp) {
        ret.add(p);
      }
    }
    return ret;
  }

  static void printProgress(uint64_t iteration, const Cost &cost,
                            const std::vector<Mesh2d> &parts,
                            const std::vector<std::vector<Vec2d>> &buffers) {
    Box bb = bounds(buffers);

    if (iteration % 100 == 0) {
      std::cout << "iteration:" << iteration << " cost:";
      for (size_t i = 0; i < cost.size(); i++) {
        if (i > 0) {
          std::cout << ",";
        }
        std::cout << cost[i];
      }
      std::cout << " width:" << bb.width() << " height:" << bb.height();
      std::cout << std::endl;
    }
  }

  static void visualize(const std::vector<Pose2d> &poses,
                        const std::vector<Mesh2d> &parts,
                        const std::vector<std::vector<Vec2d>> &buffers) {
    cv::Mat_<cv::Vec3b> image(460 * 2, 640 * 2, cv::Vec3b(0, 0, 0));
    size_t s = (image.cols + image.rows) / 400;
    for (size_t i = 0; i < parts.size(); i++) {
      auto &part = parts[i];
      auto &pose = poses[i];
      auto vertex = [&](size_t j) {
        auto p = buffers[i][j];
        return cv::Point(p.x() * s + image.cols / 2,
                         p.y() * s + image.rows / 2);
      };
      for (auto &edge : part.edges) {
        cv::line(image, vertex(edge.indices[0]), vertex(edge.indices[1]),
                 cv::Scalar(255, 255, 255));
      }
    }
    cv::imshow("preview", image);
    int key = cv::waitKey(1);
    std::cout << "key " << key << std::endl;
    if (key == 113 || key == 27) {
      exit(0);
    }
  }

  static Vec2d rot90(const Vec2d &v) { return Vec2d(v.y(), -v.x()); }

  static double computeTriangleDistance(const Vec2d &a0, const Vec2d &a1,
                                        const Vec2d &a2, const Vec2d &b0,
                                        const Vec2d &b1, const Vec2d &b2) {
    std::array<Vec2d, 6 + 9> axes;

    size_t i = 0;

    axes[i++] = rot90(a1 - a0);
    axes[i++] = rot90(a2 - a1);
    axes[i++] = rot90(a0 - a2);

    axes[i++] = rot90(b1 - b0);
    axes[i++] = rot90(b2 - b1);
    axes[i++] = rot90(b0 - b2);

    axes[i++] = a0 - b0;
    axes[i++] = a0 - b1;
    axes[i++] = a0 - b2;

    axes[i++] = a1 - b0;
    axes[i++] = a1 - b1;
    axes[i++] = a1 - b2;

    axes[i++] = a2 - b0;
    axes[i++] = a2 - b1;
    axes[i++] = a2 - b2;

    double distance = -std::numeric_limits<double>::max();

    for (auto axis : axes) {
      axis = axis.normalized();

      double da0 = axis.dot(a0);
      double da1 = axis.dot(a1);
      double da2 = axis.dot(a2);

      double db0 = axis.dot(b0);
      double db1 = axis.dot(b1);
      double db2 = axis.dot(b2);

      double alo = std::min(std::min(da0, da1), da2);
      double ahi = std::max(std::max(da0, da1), da2);
      double amid = (alo + ahi) * 0.5;
      double alen = ahi - alo;

      double blo = std::min(std::min(db0, db1), db2);
      double bhi = std::max(std::max(db0, db1), db2);
      double bmid = (blo + bhi) * 0.5;
      double blen = bhi - blo;

      double dist = (amid - bmid) - (alen + blen) * 0.5;

      distance = std::max(distance, dist);
    }

    return distance;
  }

  static double computeMeshIntersectionCost(
      const Mesh2d &mesh_a, const std::vector<Vec2d> &buffer_a,
      const Mesh2d &mesh_b, const std::vector<Vec2d> &buffer_b) {
    double meshbox_intersection =
        Box::Intersection(bounds(buffer_a).padded(padding), bounds(buffer_b))
            .breadth();
    if (meshbox_intersection == 0.0) {
      return 0.0;
    }

    double face_cost = 0.0;

    for (size_t i_a = 0; i_a < mesh_a.faces.size(); i_a++) {
      auto point_a_0 = buffer_a[mesh_a.faces[i_a].indices[0]];
      auto point_a_1 = buffer_a[mesh_a.faces[i_a].indices[1]];
      auto point_a_2 = buffer_a[mesh_a.faces[i_a].indices[2]];

      Box box_a = Box::Empty();
      box_a.add(point_a_0);
      box_a.add(point_a_1);
      box_a.add(point_a_2);
      box_a = box_a.padded(padding);

      for (size_t i_b = 0; i_b < mesh_b.faces.size(); i_b++) {
        auto point_b_0 = buffer_b[mesh_b.faces[i_b].indices[0]];
        auto point_b_1 = buffer_b[mesh_b.faces[i_b].indices[1]];
        auto point_b_2 = buffer_b[mesh_b.faces[i_b].indices[2]];

        Box box_b = Box::Empty();
        box_b.add(point_b_0);
        box_b.add(point_b_1);
        box_b.add(point_b_2);

        double tribox_intersection = Box::Intersection(box_a, box_b).breadth();
        if (tribox_intersection == 0) {
          continue;
        }

        double tri_dist =
            -computeTriangleDistance(point_a_0, point_a_1, point_a_2, point_b_0,
                                     point_b_1, point_b_2) +
            padding;

        face_cost = std::max(face_cost, tri_dist);
      }
    }

    return face_cost;
  }

  static double computeIntersectionCost(
      const std::vector<Mesh2d> &parts,
      const std::vector<std::vector<Vec2d>> &buffers) {
    double cost = 0.0;
    for (size_t i = 0; i < parts.size(); i++) {
      for (size_t j = i + 1; j < parts.size(); j++) {
        cost += computeMeshIntersectionCost(parts[i], buffers[i], parts[j],
                                            buffers[j]);
      }
    }
    return cost;
  }

  static double computeAreaCost(
      const std::vector<Mesh2d> &parts,
      const std::vector<std::vector<Vec2d>> &buffers) {
    Box bounds = Box::Empty();
    for (size_t i = 0; i < parts.size(); i++) {
      for (size_t j = 0; j < buffers[i].size(); j++) {
        bounds.add(buffers[i][j]);
      }
    }
    return bounds.padded(padding).area();
  }

  static double computeCenterCost(
      const std::vector<Mesh2d> &parts,
      const std::vector<std::vector<Vec2d>> &buffers) {
    double cost = 0;
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    size_t n = 0;
    for (auto &buffer : buffers) {
      for (auto &p : buffer) {
        center += p;
        n++;
      }
    }
    center *= 1.0 / n;
    for (auto &buffer : buffers) {
      for (auto &p : buffer) {
        cost += (p - center).squaredNorm();
      }
    }
    return cost * (1.0 / n);
  }

  static Cost computeCost(const std::vector<Mesh2d> &parts,
                          const std::vector<std::vector<Vec2d>> &buffers) {
    return Cost({
        computeIntersectionCost(parts, buffers),
        computeCenterCost(parts, buffers) * 0.5 +
            computeAreaCost(parts, buffers),
        computeCenterCost(parts, buffers),
    });
  }

  template <class RNG>
  static void shuffle(RNG &rng, std::vector<Pose2d> &poses) {
    double exp = std::uniform_real_distribution<double>(0.0, 2.0)(rng);
    double scale = 10.0 * std::pow(0.1, exp);

    std::normal_distribution<double> rdist(0.0, step_rotation * scale);
    std::normal_distribution<double> tdist(0.0, step_translation * scale);

    std::uniform_int_distribution<size_t> idist(0, poses.size() - 1);

    switch (std::uniform_int_distribution<size_t>(0, 8)(rng)) {
      case 0: {
        auto &pose = poses[idist(rng)];
        pose.rotate(M_PI * 0.5 *
                    std::uniform_int_distribution<int>(-1, +2)(rng));
        break;
      }

      case 1: {
        auto &pose_a = poses[idist(rng)];
        auto &pose_b = poses[idist(rng)];
        Vec2d pos_a = pose_a.translation();
        Vec2d pos_b = pose_b.translation();
        pose_a.translation() = pos_b;
        pose_b.translation() = pos_a;
        break;
      }

      case 2: {
        double angle = rdist(rng);
        for (auto &pose : poses) {
          pose.rotate(angle);
        }
        break;
      }

      case 3: {
        double angle =
            std::uniform_real_distribution<double>(0.0, M_PI * 2)(rng);
        for (auto &pose : poses) {
          pose.prerotate(angle);
        }
        break;
      }

      case 4: {
        auto &pose_a = poses[idist(rng)];
        auto &pose_b = poses[idist(rng)];
        pose_a.linear() = pose_b.linear();
        pose_a.rotate(M_PI * std::uniform_int_distribution<int>(0, 1)(rng));
        break;
      }

      case 5: {
        auto &pose_a = poses[idist(rng)];
        auto &pose_b = poses[idist(rng)];
        std::swap(pose_a, pose_b);
        break;
      }

      case 6: {
        auto &pose_a = poses[idist(rng)];
        auto &pose_b = poses[idist(rng)];
        std::swap(pose_a, pose_b);
        pose_a.rotate(rdist(rng));
        pose_b.rotate(rdist(rng));
        auto t = Vec2d(tdist(rng), tdist(rng));
        pose_a.translate(t);
        pose_b.translate(t);
        break;
      }

      case 7: {
        auto &pose = poses[idist(rng)];
        pose.linear().setIdentity();
        pose.rotate(M_PI * 0.5 *
                    std::uniform_int_distribution<int>(-1, +2)(rng));
        break;
      }

      case 8: {
        auto &pose_a = poses[idist(rng)];
        auto &pose_b = poses[idist(rng)];
        Vec2d center = (pose_a.translation() + pose_b.translation()) * 0.5;
        pose_a.translation() -= center;
        pose_b.translation() -= center;
        double angle =
            M_PI * 0.5 * std::uniform_int_distribution<int>(0, +3)(rng);
        pose_a.rotate(angle);
        pose_b.rotate(angle);
        center.x() += tdist(rng);
        center.y() += tdist(rng);
        pose_a.translation() += center;
        pose_b.translation() += center;
        break;
      }
    }

    {
      Vec2d center = Vec2d::Zero();
      for (auto &pose : poses) {
        center += pose.translation();
      }
      center *= 1.0 / poses.size();
      for (auto &pose : poses) {
        pose.translation() -= center;
      }
    }
  }

  template <class RNG>
  static void mutate(RNG &rng, std::vector<Pose2d> &poses) {
    double exp = std::uniform_real_distribution<double>(0.0, 2.0)(rng);
    double scale = 10.0 * std::pow(0.1, exp);

    {
      std::normal_distribution<double> rdist(0.0, step_rotation * scale);
      std::normal_distribution<double> tdist(0.0, step_translation * scale);

      std::uniform_int_distribution<size_t> idist(0, poses.size() - 1);

      switch (std::uniform_int_distribution<size_t>(0, 3)(rng)) {
        case 0:
        case 1: {
          auto &pose = poses[idist(rng)];
          pose.rotate(rdist(rng));
          pose.translate(Vec2d(tdist(rng), tdist(rng)));
          break;
        }

        case 2: {
          auto &pose_a = poses[idist(rng)];
          auto &pose_b = poses[idist(rng)];
          pose_a.rotate(rdist(rng));
          pose_b.rotate(rdist(rng));
          auto t = Vec2d(tdist(rng), tdist(rng));
          pose_a.translate(t);
          pose_b.translate(t);
          break;
        }

        case 3: {
          for (auto &pose : poses) {
            pose.rotate(rdist(rng));
            pose.translate(Vec2d(tdist(rng), tdist(rng)));
          }
          break;
        }
      }
    }

    {
      Vec2d center = Vec2d::Zero();
      for (auto &pose : poses) {
        center += pose.translation();
      }
      center *= 1.0 / poses.size();
      for (auto &pose : poses) {
        pose.translation() -= center;
      }
    }
  }

  template <class RNG>
  static void shuffleMutate(RNG &rng, std::vector<Pose2d> &poses) {
    bool doshuffle = std::uniform_int_distribution<int>(0, 1)(rng);
    if (doshuffle) {
      shuffle(rng, poses);
    } else {
      mutate(rng, poses);
    }
  }

  void solveAnnealing(std::vector<Pose2d> &poses,
                      const std::vector<Mesh2d> &parts,
                      std::vector<std::vector<Vec2d>> &buffers) {
    auto temp_buffers = buffers;

    std::default_random_engine rng{std::random_device()()};

    transform(poses, parts, buffers);
    auto cost = computeCost(parts, buffers);

    auto temp_poses = poses;

    double temperature = 1;
    std::uniform_real_distribution<double> temp_dist(0.0, 1.0);

    size_t iteration_count = 100000;
    double tmin = 1e-5;
    double decay = std::pow(tmin, 1.0 / iteration_count);

    auto vistime =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(250);

    Status status;
    for (uint64_t iter = 0; iter < iteration_count; iter++) {
      temperature = std::pow(decay, iter);

      if (std::chrono::steady_clock::now() >= vistime) {
        visualize(poses, parts, buffers);
        vistime += std::chrono::milliseconds(100);
      }

      temp_poses = poses;
      bool doshuffle = false;
      if (1) {
        doshuffle = (std::uniform_int_distribution<int>(0, 1)(rng) &&
                     ((iter * 2 < iteration_count) || (cost[0] == 0)));
        if (doshuffle) {
          shuffle(rng, temp_poses);
        } else {
          mutate(rng, temp_poses);
        }
      }
      if (0) {
        mutate(rng, temp_poses);
      }
      transform(temp_poses, parts, temp_buffers);
      auto temp_cost = computeCost(parts, temp_buffers);
      if (temp_cost <= cost || ((temp_dist(rng) < temperature) && !doshuffle &&
                                temp_cost[0] <= cost[0])) {
        cost = temp_cost;
        poses = temp_poses;
        buffers = temp_buffers;
      }

      if (_callback) {
        status.current_iteration = iter;
        status.iteration_count = iteration_count;
        status.temperature = temperature;
        status.cost = cost;
        status.poses = poses;
        _callback(status);
      }

      printProgress(iter, cost, parts, buffers);
    }
  }

  template <class RNG>
  void crossover(RNG &rng, const std::vector<Pose2d> &parent_a,
                 const std::vector<Pose2d> &parent_b,
                 std::vector<Pose2d> &child) {
    std::uniform_int_distribution<size_t> dist(0, 1);
    for (size_t i = 0; i < parent_a.size(); i++) {
      child[i] = (dist(rng) ? parent_a[i] : parent_b[i]);
    }
  }

  void solveEvolution(std::vector<Pose2d> &poses,
                      const std::vector<Mesh2d> &parts,
                      std::vector<std::vector<Vec2d>> &buffers) {
    constexpr size_t population_size = 64;
    constexpr size_t elite_count = 3;
    constexpr size_t parent_count = 64;
    constexpr size_t iteration_count = 100000;

    std::default_random_engine rng{std::random_device()()};

    struct Solution {
      std::vector<Pose2d> poses;
      Cost fitness;
      std::vector<std::vector<Vec2d>> buffers;
    };

    std::vector<Solution> population;
    {
      Solution s;
      s.poses = poses;
      s.buffers = buffers;
      population.resize(population_size, s);
    }

    auto parent_population = population;

    auto parent_dist = [](std::default_random_engine &rng) {
      std::uniform_int_distribution<size_t> dist0(0, parent_count - 1);
      size_t i = dist0(rng);
      std::uniform_int_distribution<size_t> dist1(0, i);
      return dist1(rng);
    };

    for (uint64_t iter = 0; iter < iteration_count; iter++) {
#pragma omp parallel for
      for (size_t i = 0; i < population_size; i++) {
        auto &s = population[i];
        transform(s.poses, parts, s.buffers);
        s.fitness = computeCost(parts, s.buffers);
      }
      std::sort(population.begin(), population.end(),
                [](const Solution &a, const Solution &b) {
                  return a.fitness < b.fitness;
                });

      printProgress(iter, population.front().fitness, parts,
                    population.front().buffers);

      if (iter % 100 == 0) {
        visualize(poses, parts, population.front().buffers);
      }

      parent_population = population;

      for (size_t i_child = elite_count; i_child < population_size; i_child++) {
        auto &child = population[i_child];
        auto &parent_a = parent_population[parent_dist(rng)];
        auto &parent_b = parent_population[parent_dist(rng)];

        child = parent_a;

        {
          size_t n = std::uniform_int_distribution<int>(1, 2)(rng);
          for (size_t i = 0; i < n; i++) {
            bool doshuffle = std::uniform_int_distribution<int>(0, 1)(rng);
            if (doshuffle) {
              shuffle(rng, child.poses);
            } else {
              mutate(rng, child.poses);
            }
          }
        }
      }
    }

    poses = population.front().poses;
  }

  void solveLocal(std::vector<Pose2d> &poses, const std::vector<Mesh2d> &parts,
                  std::vector<std::vector<Vec2d>> &buffers) {
    auto temp_buffers = buffers;

    std::default_random_engine rng{std::random_device()()};

    transform(poses, parts, buffers);
    auto cost = computeCost(parts, buffers);

    auto temp_poses = poses;

    size_t iteration_count = 100000;
    Status status;
    for (uint64_t iter = 0; iter < iteration_count; iter++) {
      if (iter % 1000 == 0) {
        visualize(poses, parts, buffers);
      }

      temp_poses = poses;
      shuffleMutate(rng, temp_poses);
      transform(temp_poses, parts, temp_buffers);
      auto temp_cost = computeCost(parts, temp_buffers);
      if (temp_cost <= cost) {
        cost = temp_cost;
        poses = temp_poses;
        buffers = temp_buffers;
      }

      printProgress(iter, cost, parts, buffers);

      if (_callback) {
        status.current_iteration = iter;
        status.iteration_count = iteration_count;
        status.cost = cost;
        status.poses = poses;
        _callback(status);
      }
    }
  }

  void solveTree(std::vector<Pose2d> &poses, const std::vector<Mesh2d> &parts,
                 std::vector<std::vector<Vec2d>> &buffers) {
    auto temp_buffers = buffers;

    std::default_random_engine rng{std::random_device()()};

    transform(poses, parts, buffers);
    auto cost = computeCost(parts, buffers);

    auto temp_poses = poses;
    auto temp_poses_2 = poses;

    size_t iteration_count = 100000;
    Status status;
    for (uint64_t iter = 0; iter < iteration_count; iter++) {
      if (iter % 1000 == 0) {
        visualize(poses, parts, buffers);
      }

      temp_poses = poses;
      for (size_t i = 0; i < 10; i++) {
        temp_poses_2 = temp_poses;
        shuffleMutate(rng, temp_poses_2);
        transform(temp_poses_2, parts, temp_buffers);
        auto c = computeCost(parts, temp_buffers);
        if (c[0] <= cost[0]) {
          temp_poses = temp_poses_2;
        }
      }
      transform(temp_poses, parts, temp_buffers);
      auto temp_cost = computeCost(parts, temp_buffers);
      if (temp_cost <= cost) {
        cost = temp_cost;
        poses = temp_poses;
        buffers = temp_buffers;
      }

      printProgress(iter, cost, parts, buffers);

      if (_callback) {
        status.current_iteration = iter;
        status.iteration_count = iteration_count;
        status.cost = cost;
        status.poses = poses;
        _callback(status);
      }
    }
  }

  void solveX(std::vector<Pose2d> &poses, const std::vector<Mesh2d> &parts,
              std::vector<std::vector<Vec2d>> &buffers) {
    auto temp_buffers = buffers;

    std::default_random_engine rng{std::random_device()()};

    transform(poses, parts, buffers);
    auto cost = computeCost(parts, buffers);

    auto temp_poses = poses;

    size_t iteration_count = 500000;

    double tmin = 1e-5;
    double decay = std::pow(tmin, 1.0 / iteration_count);

    Status status;
    for (uint64_t iter = 0; iter < iteration_count; iter++) {
      double temperature = std::pow(decay, iter);

      if (iter % 1000 == 0) {
        visualize(poses, parts, buffers);
      }

      {
        std::normal_distribution<double> tdist(0.0, temperature * 1);
        std::uniform_int_distribution<size_t> idist(0, poses.size() - 1);
        temp_poses = poses;
        {
          auto &pose = temp_poses[idist(rng)];
          pose.translate(Vec2d(tdist(rng), tdist(rng)));
        }
        transform(temp_poses, parts, temp_buffers);
        auto temp_cost = computeCost(parts, temp_buffers);
        if (temp_cost[0] <= cost[0]) {
          cost = temp_cost;
          poses = temp_poses;
          buffers = temp_buffers;
        }
      }

      temp_poses = poses;
      shuffleMutate(rng, temp_poses);
      transform(temp_poses, parts, temp_buffers);
      auto temp_cost = computeCost(parts, temp_buffers);
      if (temp_cost <= cost) {
        cost = temp_cost;
        poses = temp_poses;
        buffers = temp_buffers;
      }

      printProgress(iter, cost, parts, buffers);

      if (_callback) {
        status.current_iteration = iter;
        status.iteration_count = iteration_count;
        status.cost = cost;
        status.poses = poses;
        _callback(status);
      }
    }
  }

  void pack(std::vector<Mesh2d> &parts) {
    std::vector<std::vector<Vec2d>> buffers;
    for (auto &part : parts) {
      buffers.emplace_back(part.vertices.size());
    }

    std::vector<Pose2d> poses(parts.size(), Pose2d::Identity());

    for (size_t i = 0; i < parts.size(); i++) {
      auto &part = parts[i];
      Vec2d center = Vec2d::Zero();
      for (auto &vertex : part.vertices) {
        center += vertex;
      }
      center *= 1.0 / part.vertices.size();
      for (auto &vertex : part.vertices) {
        vertex -= center;
      }
      poses[i].translate(center);
    }

    for (size_t i = 0; i < parts.size(); i++) {
      auto &part = parts[i];
      Eigen::MatrixXd data(part.vertices.size(), 2);
      for (size_t i = 0; i < part.vertices.size(); i++) {
        data.row(i) = part.vertices[i];
      }
      Eigen::EigenSolver<Eigen::MatrixXd> es(data.transpose() * data);
      std::cout << "pca " << es.eigenvectors() << std::endl;
      Pose2d pose = Pose2d::Identity();
      pose.matrix().block(0, 0, 2, 2) = es.eigenvectors().real().inverse();
      if (es.eigenvalues()[0].real() > es.eigenvalues()[1].real()) {
        pose.rotate(M_PI * 0.5);
      }
      for (auto &p : part.vertices) {
        p = pose * p;
      }
    }

    solveAnnealing(poses, parts, buffers);

    for (size_t i = 0; i < parts.size(); i++) {
      transform(poses[i], parts[i], parts[i].vertices);
    }
  }
};
