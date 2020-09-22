#ifndef _KINEMATIC_CAR_
#define _KINEMATIC_CAR_

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace oc = ompl::control;
namespace ob = ompl::base;

namespace KinematicCar {
// as in
// https://ompl.kavrakilab.org/RigidBodyPlanningWithODESolverAndControls_8cpp_source.html
inline void kinematicCarODE(const oc::ODESolver::StateType& q,
                     const oc::Control* control,
                     oc::ODESolver::StateType& qdot) {
  global::settings.ompl.steering_timer.resume();
  double* u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
  double heading = q[2];
  qdot.resize(q.size(), 0);

  // Dynamics
  qdot[0] = u[0] * cos(heading);
  qdot[1] = u[0] * sin(heading);
  qdot[2] = u[0] * tan(u[1]) / global::settings.forwardpropagation.car_length;
  global::settings.ompl.steering_timer.stop();
}

// as in
// https://ompl.kavrakilab.org/RigidBodyPlanningWithODESolverAndControls_8cpp_source.html
inline void kinematicCarPostIntegration(const ob::State* /*state*/,
                                 const oc::Control* /*control*/,
                                 const double /*duration*/, ob::State* result) {
  // Normalize orientation between 0 and 2*pi
  ob::SO2StateSpace SO2;
  SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()
                        ->as<ob::SO2StateSpace::StateType>(1));
}

inline void propagate(const oc::SpaceInformation* si, const ob::State* state,
               const oc::Control* control, const double duration,
               ob::State* result) {
  global::settings.ompl.steering_timer.resume();
  static double timeStep = global::settings.forwardpropagation.dt;
  int nsteps = ceil(duration / timeStep);
  double dt = duration / nsteps;
  const double* u =
      control->as<oc::RealVectorControlSpace::ControlType>()->values;

  ob::SE2StateSpace::StateType& se2 =
      *result->as<ob::SE2StateSpace::StateType>();

  si->getStateSpace()->copyState(result, state);

  for (int i = 0; i < nsteps; i++) {
    se2.setX(se2.getX() + dt * u[0] * cos(se2.getYaw()));
    se2.setY(se2.getY() + dt * u[0] * sin(se2.getYaw()));
    se2.setYaw(se2.getYaw() +
               dt * u[0] * tan(dt * u[1]) /
                   global::settings.forwardpropagation.car_length);

    if (!si->satisfiesBounds(result)) {
      global::settings.ompl.steering_timer.stop();
      return;
    }
  }

  ob::SO2StateSpace SO2;
  SO2.enforceBounds(se2.as<ob::SO2StateSpace::StateType>(1));
  global::settings.ompl.steering_timer.stop();
}

}  // namespace KinematicCar
#endif /* _KINEMATIC_CAR_ */