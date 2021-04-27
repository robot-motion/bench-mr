## Velocity-dependent Metrics

The trajectory evaluation metrics in this folder
depend on the speed of each state of the trajectory.

At this moment, this feature is not available since the
state `ompl::base::State` is assumed to be a
`ompl::state::SE2StateSpace::StateType` and no velocity
information is available. 
