#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include <cmath>
#include <random>

#include "Eigen-3.3/Eigen/Dense"
#include "polynomial.h"
#include "planner.h"
#include "various_structs.h"

using namespace std;


struct TrajectoryGenerator
{
    double sigma = 0.2;        // use 20% of the goal value as a standard deviation
    double timestep = 0.2;     // in seconds
    double planDuration = 1.0; // in seconds. the original plan duration
    double numberOfSamples = 10;

    Trajectory generateTrajectory(StateGoal &s, Vehicle &car, Road &r);
    vector<StateGoal> perturbGoal(StateGoal &goalState);
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
};

#endif //! TRAJECTORY_GENERATOR_H
