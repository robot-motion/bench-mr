#pragma once

#include <cassert>
#include <fstream>
#include <sstream>

#include "base/Primitives.h"

/**
 * This class loads polygons from an SVG file. Polygons must be stored as path
 * tags and only the Inkscape-style formatting is supported at the moment.
 * For examples of the supported SVG files, see files in `bin/parking`.
 */
class SvgPolygonLoader {
 public:
  static std::vector<Polygon> load(const std::string &filename) {
    std::ifstream input_file(filename);
    std::string line;
    std::vector<Polygon> polygons;
    if (input_file.fail()) {
      std::cerr << "Cannot load SVG file from " << filename
                << ". Make sure the file exists." << std::endl;
      assert(0);
      return polygons;
    }
    double offset_x = 0, offset_y = 0;
    while (getline(input_file, line)) {
      trim(line);
      if (line.substr(0, 21) == "transform=\"translate(") {
        std::stringstream ss(line.substr(21, line.length() - 21));
        ss >> offset_x;
        char c;
        ss >> c;
        ss >> offset_y;
        std::cout << "SVG group: using offset_x=" << offset_x
                  << "  offset_y=" << offset_y << std::endl;
      } else if (line.substr(0, 2) == "d=") {
        Polygon poly =
            Polygon::loadFromSvgPathStr(line.substr(3, line.length() - 3));
        poly.translate(Point(offset_x, -offset_y));  // flip offset y
        if (!poly.isConvex()) {
          std::cerr << "Warning: found nonconvex polygon in " << filename
                    << ". Using its convex hull instead." << std::endl;
          poly = poly.convexHull();
          // std::cout << poly << std::endl;
          std::cerr << std::boolalpha << "Hull is convex? " << poly.isConvex()
                    << std::endl;
          assert(poly.isConvex());
        }
        polygons.emplace_back(poly);
      }
    }
    return polygons;
  }

 private:
  static inline void trim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                                    [](int ch) { return !std::isspace(ch); }));
    s.erase(std::find_if(s.rbegin(), s.rend(),
                         [](int ch) { return !std::isspace(ch); })
                .base(),
            s.end());
  }
};
