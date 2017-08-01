#ifndef PLANNER_H
#define PLANNER_H

#include <map>
#include <vector>
#include <string>
#include "various_structs.h"
#include "vehicle.h"
#include "road.h"

using namespace std;

struct Planner
{
    double nearBuffer = 4; // need for safety a buffer from front and behind cars
    double maxCost = 100000.0;
    double planDuration = 5.0; // I plan ahead for that time.

    static map<string, vector<string>> next_modes;

    string select_mode(Vehicle &car, Road &r);
    double costLaneChangeLeft(Vehicle &car, Road &r);
    double costLaneChangeRight(Vehicle &car, Road &r);
    double costKeepLane(Vehicle &car, Road &r);
    double costMatchFrontSpeed(Vehicle &car, Road &r);
    double costSpeed(Vehicle &car, double desiredSpeed, double freeRoadAhead, double speedCarInFront);
    double costSpace(Vehicle &car, double spaceNeeded, double spaceInFront, double spaceBehind);

    // realize modes by creating start and end goals for s and d
    StateGoal realizePlan(string mode, Vehicle &car, Road &r);
    StateGoal realizeKeepLane(Vehicle &car, Road &r);
    StateGoal realizeMatchFront(Vehicle &car, Road &r);
    StateGoal realizeChangeLeft(Vehicle &car, Road &r);
    StateGoal realizeChangeRight(Vehicle &car, Road &r);

    vector<double> endGoalFromTargetVelocity(Vehicle &car, Road&r , double targetVelocity);
};

#endif // !PLANNER_H