#pragma once

#include <fstream>

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
    while (getline(input_file, line)) {
      trim(line);
      if (line.substr(0, 2) == "d=")
        polygons.emplace_back(
            Polygon::loadFromSvgPathStr(line.substr(3, line.length() - 3)));
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
