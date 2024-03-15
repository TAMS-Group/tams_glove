// (c) 2021-2024 Philipp Ruppel

#include "contour.hpp"
#include "mesh.hpp"
#include "obj.hpp"
#include "pack.hpp"
#include "svg.hpp"
#include "unwrap.hpp"
#include <glob.h>

#include <iostream>

#include <boost/program_options.hpp>
#include <chrono>
#include <filesystem>
#include <fnmatch.h>
#include <shared_mutex>
#include <thread>

std::vector<std::string> xglob(const std::vector<std::string> &patterns) {
  std::vector<std::string> ret;
  for (auto &pattern : patterns) {
    glob_t glob_result;
    memset(&glob_result, 0, sizeof(glob_result));
    if (0 != glob(pattern.c_str(), 0, nullptr, &glob_result))
      throw std::runtime_error("glob failed");
    for (size_t i = 0; i < glob_result.gl_pathc; i++)
      ret.emplace_back(glob_result.gl_pathv[i]);
    globfree(&glob_result);
  }
  return ret;
}

bool xmatch(const std::vector<std::string> &patterns, const std::string &name) {
  if (patterns.empty()) return true;
  bool ret = false;
  for (auto &pattern : patterns) {
    bool matching = (0 == fnmatch(pattern.c_str(), name.c_str(), FNM_EXTMATCH));
    std::cout << "match " << pattern << " " << name << " " << (int)matching
              << std::endl;
    if (matching) ret = true;
  }
  return ret;
}

std::string sanitize(const std::string &str) {
  std::string ret;
  for (auto &c : str) {
    ret += (std::isalnum(c) ? c : 'X');
  }
  return ret;
}

int main(int argc, char **argv) {
  bool show_help = false;

  std::vector<std::string> file_patterns, material_filters, object_filters;

  bool dont_pack = false;

  boost::program_options::options_description desc;
  desc.add_options()  //
      ("help,h", boost::program_options::bool_switch(&show_help),
       "print help message")  //
      ("file,f",
       boost::program_options::value<std::vector<std::string>>(&file_patterns)
           ->multitoken(),
       "file patterns, e.g. model.obj or ../dir/*/file*")  //
      ("material,m",
       boost::program_options::value<std::vector<std::string>>(
           &material_filters)
           ->multitoken(),
       "optional material name filter")  //
      ("object,o",
       boost::program_options::value<std::vector<std::string>>(&object_filters)
           ->multitoken(),
       "optional object name filter")  //
      ("npack,p", boost::program_options::bool_switch(&dont_pack),
       "skip packing")  //
      // ("compression", boost::program_options::value<double>(), "set

      ;

  boost::program_options::positional_options_description pdesc;
  pdesc.add("file", -1);

  try {
    boost::program_options::command_line_parser parser{argc, argv};
    parser.options(desc);
    parser.positional(pdesc);
    auto parsed_options = parser.run();
    boost::program_options::variables_map vm;
    boost::program_options::store(parsed_options, vm);
    boost::program_options::notify(vm);
  } catch (boost::program_options::error &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  if (show_help) {
    std::cout << "unwrap [fileglob...] [options...]" << std::endl;
    std::cout << desc << std::endl;
    return 0;
  }

  OBJ::Options obj_opts;
  obj_opts.material_filter = [material_filters](const std::string &name) {
    return xmatch(material_filters, name);
  };
  obj_opts.object_filter = [object_filters](const std::string &name) {
    return xmatch(object_filters, name);
  };

  for (auto &filename : xglob(file_patterns)) {
    std::cout << filename << std::endl;

    std::cout << "loading " << filename << std::endl;
    OBJ obj;
    obj.load(filename, obj_opts);
    Mesh mesh = obj.mesh();
    std::cout << "vertices " << mesh.vertices.size() << std::endl;
    std::cout << "faces " << mesh.faces.size() << std::endl;
    std::cout << "edges " << mesh.edges.size() << std::endl;
    if (mesh.empty()) {
      throw std::runtime_error("ERROR: mesh is invalid or empty");
    }

    auto parts = mesh.split();
    std::cout << "parts " << parts.size() << std::endl;
    std::vector<Unwrapper> unwrappers(parts.size());
    for (size_t ipart = 0; ipart < parts.size(); ipart++) {
      unwrappers[ipart].init(parts[ipart]);
      unwrappers[ipart].start();
    }

    size_t imsize = 300;
    size_t imcols = size_t(std::ceil(std::sqrt(parts.size() * 1.0)));
    size_t imrows = size_t(std::ceil(parts.size() * 1.0 / imcols));
    cv::Mat_<cv::Vec3b> image(imrows * imsize, imcols * imsize,
                              cv::Vec3b(0, 0, 0));
    while (true) {
      {
        bool allready = true;
        for (size_t ipart = 0; ipart < parts.size(); ipart++) {
          if (!unwrappers[ipart].ready()) {
            allready = false;
          }
        }
        if (allready) {
          break;
        }
      }
      auto timeout =
          std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
#pragma omp parallel for
      for (size_t ipart = 0; ipart < parts.size(); ipart++) {
        while (std::chrono::steady_clock::now() < timeout &&
               !unwrappers[ipart].ready()) {
          unwrappers[ipart].step();
        }
      }

      for (size_t ipart = 0; ipart < parts.size(); ipart++) {
        auto img = unwrappers[ipart].visualize(imsize);
        size_t x = ipart % imcols * imsize;
        size_t y = ipart / imcols * imsize;

        img.copyTo(image(cv::Rect(x, y, imsize, imsize)));
      }
      cv::imshow("preview", image);
      cv::waitKey(1);
    }

    for (size_t ipart = 0; ipart < parts.size(); ipart++)
      unwrappers[ipart].rescale();

    std::vector<std::vector<Eigen::Vector2d>> solutions;
    for (size_t ipart = 0; ipart < parts.size(); ipart++)
      solutions.push_back(unwrappers[ipart]._uv);

    std::vector<Mesh2d> parts2d;
    for (size_t ipart = 0; ipart < parts.size(); ipart++) {
      Mesh2d part2d;
      part2d.vertices = solutions[ipart];
      part2d.faces = parts[ipart].faces;
      part2d.edges = parts[ipart].edges;
      part2d.materials = parts[ipart].materials;
      parts2d.push_back(part2d);
    }

    if (!dont_pack && !parts2d.empty()) {
      std::cout << "packing" << std::endl;

      Packer packer;

      packer.pack(parts2d);
    }

    {
      std::cout << "writing svg" << std::endl;

      std::string fname = std::filesystem::path(filename).replace_extension("");
      for (auto &f : object_filters) {
        fname += "-" + sanitize(f);
      }
      for (auto &f : material_filters) {
        fname += "-" + sanitize(f);
      }
      fname += ".svg";

      SVGWriter svg{fname};

      for (auto part2d : parts2d) {
        std::vector<Face> filtered_faces;
        for (auto &face : part2d.faces)
          if (part2d.materials.at(face.material).name != "DELETE")
            filtered_faces.push_back(face);
        part2d.faces = filtered_faces;

        for (auto &segment2d : part2d.split())
          svg.write(findContour(segment2d.vertices, segment2d.edges));
      }
    }
  }
}
