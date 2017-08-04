#ifndef PLANNER_H
#define PLANNER_H

#include <map>
#include <vector>
#include <string>
#include "various_structs.h"
#include "vehicle.h"
#include "road.h"
#include "constants.h"

using namespace std;

struct Planner {
  double nearBuffer = constants::SAFETY_DISTANCE; // need for safety a buffer from front and behind cars
  double maxCost = constants::MAX_COST;
  int nextUpdateIn = constants::UPDATE_EVERY;

  static map<string, vector<string>> next_modes;

  // counter will be set by the planner when updating
  bool shouldUpdate() { if (nextUpdateIn == 0) { return true; } else { nextUpdateIn--; return false; } } 

  string select_mode(const Vehicle &car, const Road &r);
  double costLaneChangeLeft(const Vehicle &car, const Road &r);
  double costLaneChangeRight(const Vehicle &car, const Road &r);
  double costKeepLane(const Vehicle &car, const Road &r);
  double costMatchFrontSpeed(const Vehicle &car, const Road &r);
  double costSpeed(const Vehicle &car, double desiredSpeed, double freeRoadAhead, double speedCarInFront);
  double costSpace(const Vehicle &car, double spaceNeeded, double spaceInFront, double spaceBehind);

  // realize modes by creating start and end goals for s and d
  StateGoal realizePlan(string mode, Vehicle &car, const Road &r);
  StateGoal realizeKeepLane(Vehicle &car, const Road &r);
  StateGoal realizeMatchFront(Vehicle &car, const Road &r);
  StateGoal realizeChangeLeft(Vehicle &car, const Road &r);
  StateGoal realizeChangeRight(Vehicle &car, const Road &r);
  StateGoal realizeStartEngine(Vehicle &car, const Road &r);

  vector<double> endGoalFromTargetVelocity(const StateGoal &state, const Road &r, double targetVelocity);
};

#endif // !PLANNER_H