/**
 * @file vehicle.h
 * @brief An object to represent a Vehicle driving down a highway
 *
 * It contains the necessary function and data to communicate with
 * the planner and trajectory generator to accomplish the task of
 * driving the car down the highway.
 *
 * @author  Kostas Oreopoulos
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include "json.hpp"
#include <fstream>
#include <vector>
#include <chrono>
#include "various_structs.h"

using json = nlohmann::json;
using namespace std;

/// from mph to m/s
inline double MPH_to_MPS(double speed) { return speed * 0.44704; }

/// Class Vehicle.
struct Vehicle {
  StateGoal currentGoal; ///< includes the goal state of the car. used for generating the next state
  RoadConfiguration r; ///< basic road characteristics we are moving on
  DiscreteCurve previousCurve; ///< points not consumed by the sim from our previous plan
  double carLength = 5.0;
  double x, y, yaw, speed, acc = 0.0; ///< state in cartesian coordinates
  double s, d , v_s, v_d, local_yaw; ///< state in frenet space

  string mode = "OFF"; ///< current plan car is implementing. set by planner

  Vehicle(); ///< create an empty car
  Vehicle(double x, double y, double s, double d, double yaw, double speed); ///< create a car with initial values
  explicit Vehicle(const json j, int index = 1, bool yawInDegrees = false); ///< create a car with values from json

  void updateData(const json &j, int index = 1, bool yawInDegrees = false); ///< update an existing car from json
  void updateLocalData(const double lane_yaw); ///< calculate local yaw and speed on frenet space
  void readPreviousPath(const json &j, int index = 1);  ///< read previous path from json returned by sim
  void useRoadConfiguration(RoadConfiguration rcfg); ///< apply road configuration if you change roads
  void printVehicle(ofstream &log); ///< convinience function for debugging to file
  void printVehicle(); ///<convinience function for debugging to console

  int lag();///< how much lag was in this iteration.
  int getLane() const; ///< based on d , return the lane we are on the road
  double getTargetD(int lane) const; ///< based on the lane we want to move, get the middle of it, in d-coordinates

  // predictions
  bool collidesWith(Vehicle &other, double time = 0); ///< where the vehicle will be in t-time, in frenet coordinates
  pair<bool, int> willCollideWith(Vehicle &other, int timesteps, double dt); ///< checks collision with other car

  /**
   * where the vehicle will be in t-time, in frenet coordinates
   * @param time
   * @return The state in given time {new_s, new_d, v_s, v_d}
   */
  vector<double> getStateAt(double time) const ;
};

#endif // !VEHICLE_H