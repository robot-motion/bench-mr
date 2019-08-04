#pragma once

#include <chomp/ConstraintFactory.h>
#include <chomp/Map2D.h>
#include <chomp/chomputil.h>
#include <mzcommon/DtGrid.h>
#include <utils/Stopwatch.hpp>

#include "planners/AbstractPlanner.h"

using chomp::MatX;

class CHOMP {
 public:
  CHOMP();

  ob::PlannerStatus run(const og::PathGeometric &path);

  og::PathGeometric solution() const;
  std::vector<Point> solutionPath() const;

  double planningTime() const;

 private:
  static Map2D *_map;
  Stopwatch _timer;
  std::vector<Point> _path;

  class Map2DCHelper : public chomp::ChompCollisionHelper {
   public:
    enum {
      NUM_CSPACE = 2,
      NUM_WKSPACE =
          3,  // actually just 2 but this way I can test matrix dims better
      NUM_BODIES = 1,
    };

    const Map2D map;

    explicit Map2DCHelper(const Map2D &m)
        : ChompCollisionHelper(NUM_CSPACE, NUM_WKSPACE, NUM_BODIES), map(m) {}

    Map2DCHelper()
        : ChompCollisionHelper(NUM_CSPACE, NUM_WKSPACE, NUM_BODIES), map() {}

    double getCost(const MatX &q, size_t body_index, MatX &dx_dq,
                   MatX &cgrad) override {
      assert((q.rows() == 2 && q.cols() == 1) ||
             (q.rows() == 1 && q.cols() == 2));

      dx_dq.conservativeResize(3, 2);
      dx_dq.setZero();

      dx_dq << 1, 0, 0, 1, 0, 0;

      cgrad.conservativeResize(3, 1);

      vec3f g;
      float c = map.sampleCost(vec3f((float)q(0), (float)q(1), 0.0), g);

      cgrad << g[0], g[1], 0.0;

      return c;
    }
  };
};
