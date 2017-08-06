#ifndef PLANNER_H
#define PLANNER_H

#include <map>
#include <vector>
#include <string>
#include "various_structs.h"
#include "vehicle.h"
#include "road.h"
#include "constants.h"
#include <sstream>

using namespace std;

struct Planner {
  int nextUpdateIn = 0;
  double reportedLag = 0;
  std::ostringstream logger;

  static map<string, vector<string>> next_modes;

  // counter will be set by the planner when updating
  bool shouldUpdate() {
    if (nextUpdateIn <= constants::UPDATE_WHEN) { return true; }
    else {
      nextUpdateIn--;
      return false;
    }
  }

  string select_mode(const Vehicle &car, const Road &r);
  double costLaneChange(const Vehicle &car, const Road &r, bool left);
  double costKeepLane(const Vehicle &car, const Road &r);
  double costMatchFrontSpeed(const Vehicle &car, const Road &r);
  double costEmergencyBrake(const Vehicle &car, const Road &r);
  double costInLane(const Vehicle &car, const Road &r, const int lane);

  // realize modes by creating start and end goals for s and d
  StateGoal realizePlan(string mode, Vehicle &car, const Road &r);
  StateGoal realizeKeepLane(Vehicle &car, const Road &r);
  StateGoal realizeMatchFront(Vehicle &car, const Road &r);
  StateGoal realizeChangeLeft(Vehicle &car, const Road &r);
  StateGoal realizeChangeRight(Vehicle &car, const Road &r);
  StateGoal realizeStartEngine(Vehicle &car, const Road &r);
  StateGoal realizeEmergencyBrake(const Vehicle &car, const Road &r);

  // helper functions
  double lookAheadPenalty(const Vehicle &car, const Road &r, const int lane);
  double lookBehindPenalty(const Vehicle &car, const Road &r, const int lane);
  double lookBesidesPenalty(const Vehicle &car, const Road &r, const int offset);

  vector<double> endGoalFromTargetVelocity(const StateGoal &state, const Road &r, double targetVelocity);

  void logSelectedPlan(const Vehicle &car, const Road &r, const StateGoal &goal, string selectedPlan);
};

#endif // !PLANNER_H