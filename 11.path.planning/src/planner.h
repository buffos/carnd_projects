#ifndef PLANNER_H
#define PLANNER_H

#include <map>
#include <vector>
#include <string>
#include "vehicle.h"
#include "road.h"

using namespace std;

struct Planner {
    double nearBuffer = 4; // need for safety a buffer from front and behind cars
    double maxCost = 100000.0;

    static map<string, vector<string> > next_modes;

    void update_mode(Vehicle &car, Road &r);
    double costLaneChangeLeft(Vehicle &car, Road &r);
    double costLaneChangeRight(Vehicle &car, Road &r);
    double costKeepLane(Vehicle &car, Road &r);

};


#endif // !PLANNER_H