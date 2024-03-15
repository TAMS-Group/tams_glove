// GloveWise
// (c) 2023 Philipp Ruppel

#include <iostream>

#include <Eigen/Dense>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>

#include <visualization_msgs/MarkerArray.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>

#include <optional>
#include <array>

#include <X11/XKBlib.h>

namespace py = pybind11;

namespace glovewise {

struct Skinning {
  std::vector<Eigen::Vector4f> vertices;
  std::vector<std::vector<size_t>> indices;
  std::vector<std::vector<float>> weights;
  std::vector<Eigen::Matrix4f> binding;

  std::vector<Eigen::Vector3f> compute(
      const std::vector<Eigen::Matrix4f>& poses) {
    std::vector<Eigen::Vector3f> ret(vertices.size(), Eigen::Vector3f::Zero());
    for (size_t ibone = 0; ibone < binding.size(); ibone++) {
      auto& bone_weights = weights.at(ibone);
      auto& bone_indices = indices.at(ibone);
      Eigen::Matrix<float, 3, 4> bone_matrix =
          (poses[ibone] * binding[ibone]).block(0, 0, 3, 4);
      size_t nweights = bone_weights.size();
      for (size_t iweight = 0; iweight < nweights; iweight++) {
        ret[bone_indices[iweight]] +=
            (bone_matrix * vertices[bone_indices[iweight]]) *
            bone_weights[iweight];
      }
    }
    return ret;
  }
};

template <class AvgType, class RowType>
bool match_descriptors(AvgType& avg, float threshold,
                       const std::initializer_list<RowType>& feature_rows) {
  {
    auto feature_it = feature_rows.begin();
    avg = *feature_it;
    while (true) {
      feature_it++;
      if (feature_it == feature_rows.end()) {
        break;
      }
      avg += *feature_it;
    }
    avg *= (1.0f / feature_rows.size());
  }
  for (auto& row : feature_rows) {
    float dist = (avg - row.transpose()).squaredNorm();
    if (dist > threshold * threshold) {
      return false;
    }
  }
  return true;
}

bool match_sizes(float threshold, const std::initializer_list<float>& sizes) {
  float avg_size = 0;
  {
    auto feature_it = sizes.begin();
    avg_size = *feature_it;
    while (true) {
      feature_it++;
      if (feature_it == sizes.end()) {
        break;
      }
      avg_size += *feature_it;
    }
    avg_size *= (1.0f / sizes.size());
  }
  for (auto& size : sizes) {
    if (std::max(size, avg_size) > threshold * std::min(size, avg_size)) {
      return false;
    }
  }
  return true;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXf> triangulate(
    const Eigen::MatrixXd& cam_pos,
    const std::vector<Eigen::MatrixXd>& feature_cam_u,
    const std::vector<Eigen::MatrixXd>& feature_cam_v,
    const std::vector<Eigen::VectorXf>& feature_sizes,
    const std::vector<Eigen::MatrixXf>& feature_descriptors,
    double max_ray_error, float max_feature_descriptor_distance,
    float max_size_ratio) {
  size_t cam_count = cam_pos.rows();
  size_t feature_dimensions = feature_descriptors.at(0).cols();

  std::vector<double> ret_pos;
  std::vector<float> ret_desc;

  Eigen::MatrixXd gradients = Eigen::MatrixXd::Zero(6, 3);
  Eigen::VectorXd residuals = Eigen::VectorXd::Zero(6);

  Eigen::VectorXf feature_avg;

  for (size_t cam_i = 0; cam_i < cam_count; cam_i++) {
    Eigen::Vector3d i_p = cam_pos.row(cam_i);
    for (size_t row_i = 0; row_i < feature_cam_u.at(cam_i).rows(); row_i++) {
      for (size_t cam_j = cam_i + 1; cam_j < cam_count; cam_j++) {
        Eigen::Vector3d j_p = cam_pos.row(cam_j);
        for (size_t row_j = 0; row_j < feature_cam_u.at(cam_j).rows();
             row_j++) {
          {
            bool match =
                match_descriptors(feature_avg, max_feature_descriptor_distance,
                                  {
                                      feature_descriptors[cam_i].row(row_i),
                                      feature_descriptors[cam_j].row(row_j),
                                  });
            if (!match) continue;
          }

          for (size_t cam_k = cam_j + 1; cam_k < cam_count; cam_k++) {
            Eigen::Vector3d k_p = cam_pos.row(cam_k);
            for (size_t row_k = 0; row_k < feature_cam_u.at(cam_k).rows();
                 row_k++) {
              {
                bool match = match_descriptors(
                    feature_avg, max_feature_descriptor_distance,
                    {
                        feature_descriptors[cam_i].row(row_i),
                        feature_descriptors[cam_j].row(row_j),
                        feature_descriptors[cam_k].row(row_k),
                    });
                if (!match) continue;
              }

              {
                float s_i = feature_sizes.at(cam_i)[row_i];
                float s_j = feature_sizes.at(cam_j)[row_j];
                float s_k = feature_sizes.at(cam_k)[row_k];
                {
                  bool match = match_sizes(max_size_ratio, {s_i, s_j, s_k});
                  if (!match) continue;
                }
              }

              Eigen::Vector3d i_u = feature_cam_u.at(cam_i).row(row_i);
              Eigen::Vector3d i_v = feature_cam_v.at(cam_i).row(row_i);

              Eigen::Vector3d j_u = feature_cam_u.at(cam_j).row(row_j);
              Eigen::Vector3d j_v = feature_cam_v.at(cam_j).row(row_j);

              Eigen::Vector3d k_u = feature_cam_u.at(cam_k).row(row_k);
              Eigen::Vector3d k_v = feature_cam_v.at(cam_k).row(row_k);

              gradients.row(0) = i_u;
              gradients.row(1) = i_v;

              gradients.row(2) = j_u;
              gradients.row(3) = j_v;

              gradients.row(4) = k_u;
              gradients.row(5) = k_v;

              residuals[0] = i_u.dot(i_p);
              residuals[1] = i_v.dot(i_p);

              residuals[2] = j_u.dot(j_p);
              residuals[3] = j_v.dot(j_p);

              residuals[4] = k_u.dot(k_p);
              residuals[5] = k_v.dot(k_p);

              Eigen::Vector3d solution =
                  gradients.colPivHouseholderQr().solve(residuals);

              {
                bool max_ray_error_violated = false;
                for (size_t i = 0; i < gradients.rows() / 2; i++) {
                  Eigen::Vector2d err =
                      gradients.block(i * 2, 0, 2, 3) * solution -
                      residuals.segment(i * 2, 2);

                  double e = err.squaredNorm();
                  if (e > max_ray_error * max_ray_error) {
                    max_ray_error_violated = true;
                    break;
                  }
                }
                if (max_ray_error_violated) {
                  continue;
                }
              }

              ret_pos.push_back(solution.x());
              ret_pos.push_back(solution.y());
              ret_pos.push_back(solution.z());

              for (size_t dim = 0; dim < feature_dimensions; dim++) {
                ret_desc.push_back(feature_avg[dim]);
              }
            }
          }
        }
      }
    }
  }

  Eigen::MatrixXd ret_pos_mat = Eigen::MatrixXd::Zero(ret_pos.size() / 3, 3);
  for (size_t row = 0; row < ret_pos_mat.rows(); row++) {
    for (size_t col = 0; col < 3; col++) {
      ret_pos_mat(row, col) = ret_pos[row * 3 + col];
    }
  }

  Eigen::MatrixXf ret_desc_mat = Eigen::MatrixXf::Zero(
      ret_desc.size() / feature_dimensions, feature_dimensions);
  for (size_t row = 0; row < ret_desc_mat.rows(); row++) {
    for (size_t col = 0; col < feature_dimensions; col++) {
      ret_desc_mat(row, col) = ret_desc[row * feature_dimensions + col];
    }
  }

  return std::make_tuple(ret_pos_mat, ret_desc_mat);
}

std::vector<std::vector<std::pair<size_t, size_t>>> raycheck(
    const Eigen::MatrixXd& pos_mat, const Eigen::MatrixXf& desc_mat,
    const Eigen::MatrixXd& cam_pos,
    const std::vector<Eigen::MatrixXd>& feature_cam_u,
    const std::vector<Eigen::MatrixXd>& feature_cam_v,
    const std::vector<Eigen::MatrixXf>& feature_descriptors,
    double max_ray_error, float max_feature_descriptor_distance) {
  size_t cam_count = cam_pos.rows();
  size_t feature_dimensions = feature_descriptors.at(0).cols();
  Eigen::VectorXf feature_avg;
  std::vector<std::vector<std::pair<size_t, size_t>>> ret(pos_mat.rows());
  for (size_t det_i = 0; det_i < pos_mat.rows(); det_i++) {
    Eigen::Vector3d det_pos = pos_mat.row(det_i);
    auto det_desc = desc_mat.row(det_i);
    for (size_t cam_i = 0; cam_i < cam_count; cam_i++) {
      Eigen::Vector3d cpos = cam_pos.row(cam_i);
      for (size_t ft_i = 0; ft_i < feature_cam_u.at(cam_i).rows(); ft_i++) {
        {
          Eigen::Vector3d i_u = feature_cam_u.at(cam_i).row(ft_i);
          Eigen::Vector3d i_v = feature_cam_v.at(cam_i).row(ft_i);
          double du = i_u.dot(cpos - det_pos);
          double dv = i_v.dot(cpos - det_pos);
          double epos_2 = Eigen::Vector2d(du, dv).squaredNorm();
          if (epos_2 >= max_ray_error * max_ray_error) {
            continue;
          }
        }
        {
          bool match =
              match_descriptors(feature_avg, max_feature_descriptor_distance,
                                {
                                    det_desc,
                                    feature_descriptors[cam_i].row(ft_i),
                                });
          if (!match) continue;
        }
        ret[det_i].emplace_back(cam_i, ft_i);
      }
    }
  }
  return ret;
}

std::optional<std::array<double, 3>> meshline(const py::array_t<double>& mesh,
                                              const py::array_t<double>& pa,
                                              const py::array_t<double>& pb) {
  typedef CGAL::Exact_predicates_exact_constructions_kernel K;

  auto mesh_data = mesh.unchecked<2>();
  size_t face_count = mesh.shape(0) / 3;

  auto da = pa.unchecked<1>();
  auto db = pb.unchecked<1>();
  K::Line_3 lin(K::Point_3(da(0), da(1), da(2)),
                K::Point_3(db(0), db(1), db(2)));

  for (size_t iface = 0; iface < face_count; iface++) {
    K::Triangle_3 tri(  //

        K::Point_3(mesh_data(iface * 3 + 0, 0),   //
                   mesh_data(iface * 3 + 0, 1),   //
                   mesh_data(iface * 3 + 0, 2)),  //

        K::Point_3(mesh_data(iface * 3 + 1, 0),   //
                   mesh_data(iface * 3 + 1, 1),   //
                   mesh_data(iface * 3 + 1, 2)),  //

        K::Point_3(mesh_data(iface * 3 + 2, 0),  //
                   mesh_data(iface * 3 + 2, 1),  //
                   mesh_data(iface * 3 + 2, 2))  //

    );

    if (auto result = intersection(tri, lin)) {
      if (auto* ipoint = boost::get<K::Point_3>(&*result)) {
        return std::array<double, 3>({CGAL::to_double(ipoint->approx().x()),
                                      CGAL::to_double(ipoint->approx().y()),
                                      CGAL::to_double(ipoint->approx().z())});
      }
    }
  }
  return std::nullopt;
}

void init_python(py::module& m) {
  py::class_<Skinning>(m, "Skinning")
      .def(py::init<>())
      .def_readwrite("vertices", &Skinning::vertices)
      .def_readwrite("indices", &Skinning::indices)
      .def_readwrite("weights", &Skinning::weights)
      .def_readwrite("binding", &Skinning::binding)
      .def("compute",
           [](Skinning* thiz, const std::vector<Eigen::Matrix4f>& poses) {
             auto vertices = thiz->compute(poses);
             py::array_t<float> ret;
             ret.resize({vertices.size(), size_t(3)});
             {
               auto r = ret.mutable_unchecked();
               for (size_t i = 0; i < vertices.size(); i++) {
                 r(i, 0) = vertices[i].x();
                 r(i, 1) = vertices[i].y();
                 r(i, 2) = vertices[i].z();
               }
             }
             return ret;
           });

  m.def("parallel_for",
        [](int iterations, const std::function<void(int)>& callback) {
          {
            py::gil_scoped_release release;
#pragma omp parallel for
            for (int iteration = 0; iteration < iterations; iteration++) {
              py::gil_scoped_acquire acquire;
              callback(iteration);
            }
          }
        });

  m.def("build_mesh_message",
        [](const std::string& ns, const py::array_t<float>& color,
           const py::array_t<float>& vertices) {
          size_t vertex_count = vertices.shape(0);

          visualization_msgs::Marker marker;

          marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

          marker.scale.x = 1;
          marker.scale.y = 1;
          marker.scale.z = 1;

          marker.ns = ns;

          auto vertex_data = vertices.unchecked<2>();
          for (size_t i = 0; i < vertex_count; i++) {
            geometry_msgs::Point point;
            point.x = vertex_data(i, 0);
            point.y = vertex_data(i, 1);
            point.z = vertex_data(i, 2);
            marker.points.push_back(point);
          }

          auto color_data = color.unchecked<1>();
          marker.color.r = color_data(0);
          marker.color.g = color_data(1);
          marker.color.b = color_data(2);
          marker.color.a = color_data(3);

          visualization_msgs::MarkerArray array;
          array.markers.push_back(marker);

          size_t length = ros::serialization::serializationLength(array);
          std::vector<uint8_t> buffer(length);
          ros::serialization::OStream stream(buffer.data(), length);
          ros::serialization::serialize(stream, array);
          return py::bytearray((const char*)buffer.data(), buffer.size());
        });

  m.def("build_colored_mesh_message",
        [](const std::string& ns, const py::array_t<float>& colors,
           const py::array_t<float>& vertices) {
          size_t vertex_count = vertices.shape(0);

          visualization_msgs::MarkerArray array;
          array.markers.emplace_back();

          visualization_msgs::Marker& marker = array.markers.back();

          marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

          marker.scale.x = 1;
          marker.scale.y = 1;
          marker.scale.z = 1;

          marker.ns = ns;

          auto vertex_data = vertices.unchecked<2>();
          marker.points.reserve(vertex_count);
          for (size_t i = 0; i < vertex_count; i++) {
            geometry_msgs::Point point;
            point.x = vertex_data(i, 0);
            point.y = vertex_data(i, 1);
            point.z = vertex_data(i, 2);
            marker.points.push_back(point);
          }

          auto color_data = colors.unchecked<2>();
          marker.colors.reserve(vertex_count);
          for (size_t i = 0; i < vertex_count; i++) {
            std_msgs::ColorRGBA color;
            color.r = color_data(i, 0);
            color.g = color_data(i, 1);
            color.b = color_data(i, 2);
            color.a = color_data(i, 3);
            marker.colors.push_back(color);
          }

          size_t length = ros::serialization::serializationLength(array);
          std::vector<uint8_t> buffer(length);
          ros::serialization::OStream stream(buffer.data(), length);
          ros::serialization::serialize(stream, array);
          return py::bytearray((const char*)buffer.data(), buffer.size());
        });

  m.def("triangulate", triangulate);

  m.def("raycheck", raycheck);

  m.def("meshline", meshline);

  m.def("getkeys", []() {
    static Display* dpy = XOpenDisplay(nullptr);
    XkbStateRec state;
    XkbGetState(dpy, XkbUseCoreKbd, &state);
    return (int)state.mods;
  });
}

}  // namespace glovewise

PYBIND11_MODULE(pyglovewise, m) {
  m.def("test", []() { return "hello"; });
  glovewise::init_python(m);
}