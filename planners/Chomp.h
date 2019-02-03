#include <utility>

#pragma once

#include <chomp/ConstraintFactory.h>
#include <chomp/Map2D.h>
#include <chomp/chomputil.h>
#include <mzcommon/DtGrid.h>

#include "AbstractPlanner.hpp"

using chomp::MatX;

class ChompPlanner : public AbstractPlanner {
 public:
  ChompPlanner();

  std::string name() const override { return "CHOMP"; }

  ob::PlannerStatus run() override;

  std::vector<GNode> solutionTrajectory() const override;

  std::vector<Tpoint> solutionPath() const override;

  bool hasReachedGoalExactly() const override;

  double planningTime() const override;

 private:
  static Map2D *_map;
  Stopwatch _timer;
  std::vector<Tpoint> _path;

  void _initializeStraightLine(int N, const Map2D &map, const vec2f &p0,
                               const vec2f &p1, MatX &xi, MatX &q0, MatX &q1);

  void _initializeThetaStar(int N, const vec2f &p0, const vec2f &p1, MatX &xi,
                            MatX &q0, MatX &q1, bool extraClearing);

  class Map2DCHelper : public chomp::ChompCollisionHelper {
   public:
    enum {
      NUM_CSPACE = 2,
      NUM_WKSPACE =
          3,  // actually just 2 but this way I can test matrix dims better
      NUM_BODIES = 1,
    };

    const Map2D map;

    Map2DCHelper(const Map2D &m)
        : ChompCollisionHelper(NUM_CSPACE, NUM_WKSPACE, NUM_BODIES), map(m) {}

    Map2DCHelper()
        : ChompCollisionHelper(NUM_CSPACE, NUM_WKSPACE, NUM_BODIES), map() {}

    virtual double getCost(const MatX &q, size_t body_index, MatX &dx_dq,
                           MatX &cgrad) {
      assert((q.rows() == 2 && q.cols() == 1) ||
             (q.rows() == 1 && q.cols() == 2));

      dx_dq.conservativeResize(3, 2);
      dx_dq.setZero();

      dx_dq << 1, 0, 0, 1, 0, 0;

      cgrad.conservativeResize(3, 1);

      vec3f g;
      float c = map.sampleCost(vec3f(q(0), q(1), 0.0), g);

      cgrad << g[0], g[1], 0.0;

      return c;
    }
  };
};
