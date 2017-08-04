#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include <cmath>
#include <random>

#include "../src/Eigen-3.3/Eigen/Core"
#include "../src/Eigen-3.3/Eigen/QR"
#include "../src/Eigen-3.3/Eigen/Dense"
#include "polynomial.h"
#include "planner.h"
#include "various_structs.h"
#include "constants.h"

using namespace std;

struct TrajectoryGenerator {
  double sigma = constants::STD_DEVIATION;        // use 5% of the goal value as a standard deviation
  double numberOfSamples = constants::SAMPLE_TRAJECTORIES;

  TrajectoryGenerator();
  Trajectory generateTrajectory(StateGoal &s, Vehicle &car, Road &r);
  vector<StateGoal> perturbGoal(StateGoal &goalState, Road &r);
  vector<double> jmt(StateGoal &s, double time, int s_or_d = 1); // 1 is s , 2 means d

  // trajectory evaluation functions
  double totalCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);

  double timeDifferenceCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  double s_DifferenceCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  double d_DifferenceCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  double collisionCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  double bufferCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  double maxAccelerationCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  double maxJerkCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  double maxSpeedCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  double efficiencyCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  double totalAccelerationCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  double totalJerkCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  double slowLaneChangeCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
};

#endif //! TRAJECTORY_GENERATOR_H
