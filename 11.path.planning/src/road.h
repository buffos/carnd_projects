#ifndef ROAD_H
#define ROAD_H

#include "json.hpp"
#include <vector>
#include "vehicle.h"
#include "tools.h"
#include "various_structs.h"

using json = nlohmann::json;
using namespace std;

struct Road
{
    vector<Vehicle> cars;
    vector<WayPoint> wpts;
    const RoadConfiguration rcfg;

    inline Road() {}
    void updateData(json j, int index = 1);
    void readWayPointsFromFile(string filename);

    vector<double> distanceInFront(Vehicle &car, int lane);
    vector<double> distanceBehind(Vehicle &car, int lane);

    double closestVehicleAt(double s, double d, double time);
};

#endif // !ROAD_H