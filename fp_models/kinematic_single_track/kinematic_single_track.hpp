#ifndef _KINEMATIC_SINGLE_TRACK_
#define _KINEMATIC_SINGLE_TRACK_

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace oc = ompl::control;
namespace ob = ompl::base;

namespace kinematicSingleTrack {
// as in
// https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/blob/master/vehicleModels_commonRoad.pdf
void kinematicSingleTrackODE(const oc::ODESolver::StateType& q,
                             const oc::Control* control,
                             oc::ODESolver::StateType& qdot) {
  double* u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
  double heading = q[2];
  double car_length = 0.30;
  qdot.resize(q.size(), 0);
  // q = [x, y, heading, v_long, steering_angle]
  // u = [long_acc, d_steering_angle ]
  qdot[0] = q[3] * cos(heading);
  qdot[1] = q[3] * sin(heading);
  qdot[2] = q[3] * (1 / car_length) * tan(q[4]);

  qdot[3] = u[0];
  qdot[4] = u[1];
}

// as in
// https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/blob/master/vehicleModels_commonRoad.pdf
void kinematicSingleTrackPostIntegration(const ob::State* state,
                                         const oc::Control* /*control*/,
                                         const double /*duration*/,
                                         ob::State* result) {
  // Normalize orientation between 0 and 2*pi
  ob::SO2StateSpace SO2;
  auto* compState = result->as<ob::CompoundStateSpace::StateType>();
  auto* se2state = compState->as<ob::SE2StateSpace::StateType>(0);
  SO2.enforceBounds(se2state->as<ob::SO2StateSpace::StateType>(1));
}

void propagate(const oc::SpaceInformation* si, const ob::State* state,
               const oc::Control* control, const double duration,
               ob::State* result) {
  static double timeStep = .1;
  int nsteps = ceil(duration / timeStep);
  double car_length = 0.30;

  double dt = duration / nsteps;
  const double* u =
      control->as<oc::RealVectorControlSpace::ControlType>()->values;

  ob::CompoundStateSpace::StateType& s =
      *result->as<ob::CompoundStateSpace::StateType>();
  ob::SE2StateSpace::StateType& se2 = *s.as<ob::SE2StateSpace::StateType>(0);
  ob::RealVectorStateSpace::StateType& realPart =
      *s.as<ob::RealVectorStateSpace::StateType>(1);

  si->getStateSpace()->copyState(result, state);

  for (int i = 0; i < nsteps; i++) {
    se2.setX(se2.getX() + dt * realPart.values[0] * cos(se2.getYaw()));
    se2.setY(se2.getY() + dt * realPart.values[0] * sin(se2.getYaw()));
    se2.setYaw(se2.getYaw() + dt * realPart.values[0] *
                                  tan(dt * realPart.values[1]) / car_length);
    realPart.values[0] = realPart.values[0] + dt * u[0];
    realPart.values[1] = realPart.values[1] + dt * u[1];

    if (!si->satisfiesBounds(result)) return;
  }

  ob::SO2StateSpace SO2;
  SO2.enforceBounds(se2.as<ob::SO2StateSpace::StateType>(1));
}

}  // namespace kinematicSingleTrack
#endif /* _KINEMATIC_SINGLE_TRACK_ */