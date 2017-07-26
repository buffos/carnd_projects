#ifndef PLANNER_H
#define PLANNER_H

#include <map>
#include <vector>
#include <string>
#include "vehicle.h"
#include "road.h"

using namespace std;

struct StateGoal
{
    vector<double> start{0.0, 0.0, 0.0};
    vector<double> end{0.0, 0.0, 0.0};
};

struct Planner
{
    double nearBuffer = 4; // need for safety a buffer from front and behind cars
    double maxCost = 100000.0;
    double planDuration = 1.0; // I plan ahead for that time.

    static map<string, vector<string>> next_modes;

    string select_mode(Vehicle &car, Road &r);
    double costLaneChangeLeft(Vehicle &car, Road &r);
    double costLaneChangeRight(Vehicle &car, Road &r);
    double costKeepLane(Vehicle &car, Road &r);
    double costMatchFrontSpeed(Vehicle &car, Road &r);
    double costSpeed(double desiredSpeed, double freeRoadAhead, double speedCarInFront);

    // realize modes by creating start and end goals for s and d
};

#endif // !PLANNER_H