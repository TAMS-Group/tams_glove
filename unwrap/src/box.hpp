// (c) 2021-2024 Philipp Ruppel

#pragma once

#include <Eigen/Dense>

struct Box {
  double xmin = 0, xmax = 0, ymin = 0, ymax = 0;

  static Box Empty() {
    Box ret;
    ret.xmin = std::numeric_limits<double>::max();
    ret.xmax = -std::numeric_limits<double>::max();
    ret.ymin = std::numeric_limits<double>::max();
    ret.ymax = -std::numeric_limits<double>::max();
    return ret;
  }

  void add(const Eigen::Vector2d &p) {
    xmin = std::min(xmin, p.x());
    xmax = std::max(xmax, p.x());
    ymin = std::min(ymin, p.y());
    ymax = std::max(ymax, p.y());
  }

  Box padded(double padding) const {
    Box ret = *this;
    ret.xmin -= padding;
    ret.xmax += padding;
    ret.ymin -= padding;
    ret.ymax += padding;
    return ret;
  }

  double breadth() const {
    return std::min(std::max(0.0, xmax - xmin), std::max(0.0, ymax - ymin));
  }

  double length() const {
    return std::max(std::max(0.0, xmax - xmin), std::max(0.0, ymax - ymin));
  }

  double area() const {
    return std::max(0.0, xmax - xmin) * std::max(0.0, ymax - ymin);
  }

  double width() const { return std::max(0.0, xmax - xmin); }
  double height() const { return std::max(0.0, ymax - ymin); }

  static Box Intersection(const Box &a, const Box &b) {
    Box box;
    box.xmin = std::max(a.xmin, b.xmin);
    box.xmax = std::min(a.xmax, b.xmax);
    box.ymin = std::max(a.ymin, b.ymin);
    box.ymax = std::min(a.ymax, b.ymax);
    return box;
  }
};
