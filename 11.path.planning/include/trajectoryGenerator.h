/**
 * @file trajectoryGenerator.h
 * @brief An object to create polynomial curves tha link the planner start-end goals
 *
 * It contains the necessary function to create trajectories and evaluate them
 * based on defined criteria.
 *
 * @author  Kostas Oreopoulos
 */
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
  /**
   * The basic function that generates the polynomials the link the
   * goal states
   * @param s The goal state as provided by the Planner
   * @param car The vehicle we are creating trajectory for
   * @param r The road the vehicle is travelling one
   * @return A trajectory (two 5th order polynomials)
   */
  Trajectory generateTrajectory(StateGoal &s, Vehicle &car, Road &r);
  /**
   * Given the planner initial plan, the function perturbs it
   * to create similar plans that provide a better path from
   * state A to state B
   * @param goalState The initial goal state
   * @param r The road the vehicle travels in
   * @return a vector of alternate Goal States
   */
  vector<StateGoal> perturbGoal(StateGoal &goalState, Road &r);
  /**
   * An implementation of Jerk Minimizing Trajectories
   * @param s The goal state, containing start and end states
   * @param time The time duration of the trajectory
   * @param s_or_d (1) will create a trajectory for s-coordinate. (2) for d
   * @return A vector containing the coeeficients of the polynomial
   */
  vector<double> jmt(StateGoal &s, double time, int s_or_d = 1); // 1 is s , 2 means d

  /**
   * The total cost (evaluation) of the generated trajectory
   * @param tr The trajectory (polynomials)
   * @param s The state goal
   * @param car The vehicle we examine
   * @param r The Road we are travelling in
   * @return The evaluation of the trajectory
   */
  double totalCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  /**
   * Evaluation penalizing the time difference (in duration) of the generated
   * trajectory compared to the initial one
   * @param tr The trajectory (polynomials)
   * @param s The state goal
   * @param car The vehicle we examine
   * @param r The Road we are travelling in
   * @return The evaluation of the trajectory
   */
  double timeDifferenceCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  /**
   * Evaluation penalizing the difference in s coordinates of the generated
   * trajectory compared to the initial one
   * @param tr The trajectory (polynomials)
   * @param s The state goal
   * @param car The vehicle we examine
   * @param r The Road we are travelling in
   * @return The evaluation of the trajectory
   */
  double s_DifferenceCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  /**
   * Evaluation penalizing the difference in d coordinates of the generated
   * trajectory compared to the initial one
   * @param tr The trajectory (polynomials)
   * @param s The state goal
   * @param car The vehicle we examine
   * @param r The Road we are travelling in
   * @return The evaluation of the trajectory
   */
  double d_DifferenceCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  /**
   * Evaluation penalizing possible collisions with other vehicles
   * @param tr The trajectory (polynomials)
   * @param s The state goal
   * @param car The vehicle we examine
   * @param r The Road we are travelling in
   * @return The evaluation of the trajectory
   */
  double collisionCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  /**
   * Evaluation penalizing staying close to traffic
   * @param tr The trajectory (polynomials)
   * @param s The state goal
   * @param car The vehicle we examine
   * @param r The Road we are travelling in
   * @return The evaluation of the trajectory
   */
  double bufferCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  /**
   * Evaluation penalizing trajectories breaking the defined range for acceleration
   * @param tr The trajectory (polynomials)
   * @param s The state goal
   * @param car The vehicle we examine
   * @param r The Road we are travelling in
   * @return The evaluation of the trajectory
   */
  double maxAccelerationCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  /**
   * Evaluation penalizing trajectories breaking the defined range for jerk
   * @param tr The trajectory (polynomials)
   * @param s The state goal
   * @param car The vehicle we examine
   * @param r The Road we are travelling in
   * @return The evaluation of the trajectory
   */
  double maxJerkCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  /**
   * Evaluation penalizing trajectories breaking the defined range for speed
   * @param tr The trajectory (polynomials)
   * @param s The state goal
   * @param car The vehicle we examine
   * @param r The Road we are travelling in
   * @return The evaluation of the trajectory
   */
  double maxSpeedCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  /**
   * Evaluation penalizing trajectories going slow
   * @param tr The trajectory (polynomials)
   * @param s The state goal
   * @param car The vehicle we examine
   * @param r The Road we are travelling in
   * @return The evaluation of the trajectory
   */
  double efficiencyCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  /**
   * Evaluation penalizing trajectories with high average acceleration
   * @param tr The trajectory (polynomials)
   * @param s The state goal
   * @param car The vehicle we examine
   * @param r The Road we are travelling in
   * @return The evaluation of the trajectory
   */
  double totalAccelerationCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  /**
   * Evaluation penalizing trajectories with high average jerk
   * @param tr The trajectory (polynomials)
   * @param s The state goal
   * @param car The vehicle we examine
   * @param r The Road we are travelling in
   * @return The evaluation of the trajectory
   */
  double totalJerkCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
  double slowLaneChangeCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r);
};

#endif //! TRAJECTORY_GENERATOR_H
