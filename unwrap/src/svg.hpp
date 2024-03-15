// (c) 2021-2024 Philipp Ruppel

#pragma once

#include "mesh.hpp"

class SVGWriter {
  std::ofstream _ostream;

  double _page_width_mm = 210;
  double _page_height_mm = 297;

  double _uu_per_mm = 3.779527559;

  size_t _id = 0;

 public:
  SVGWriter(const std::string &filename) : _ostream(filename) {
    _ostream << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    _ostream << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
             << "xmlns:xlink=\"http://www.w3.org/1999/xlink\" version=\"1.1\" "
             << "xmlns:sodipodi=\"http://sodipodi.sourceforge.net/DTD/"
                "sodipodi-0.dtd\" "
             << "baseProfile=\"full\" width=\"" << _page_width_mm * _uu_per_mm
             << "px\" height=\"" << _page_height_mm * _uu_per_mm << "px\">\n";
    _ostream << R"(<sodipodi:namedview
id="namedview1"
showgrid="true">
<inkscape:grid
type="xygrid"
id="grid2075"
units="mm"
spacingx="1mm"
spacingy="1mm" />
</sodipodi:namedview>)";
  }

  ~SVGWriter() { _ostream << "</svg>\n"; }

  void write(const std::vector<Eigen::Vector2d> &contour) {
    _ostream
        << "<path "
        << "style=\"stroke:none;stroke-width:1;fill:#000000;fill-opacity:1\" "
        << "id=\"" << (_id++) << "\" "
        << "d=\"";

    for (size_t i = 0; i < contour.size(); i++) {
      auto &p = contour[i];
      _ostream << (i == 0 ? "M" : "L");
      _ostream << " " << (p.x() + _page_width_mm / 2) * _uu_per_mm << ","
               << (p.y() + _page_height_mm / 2) * _uu_per_mm << " ";
    }

    _ostream << "Z";
    _ostream << "\"/>\n";
  }
};
