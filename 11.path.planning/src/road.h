#ifndef ROAD_H
#define ROAD_H

#include "json.hpp"
#include <vector>
#include "vehicle.h"
#include "tools.h"

using json = nlohmann::json;
using namespace std;

struct Road
{
    vector<Vehicle> cars;
    vector<WayPoint> wpts;
    const double max_s = 6945.554;        // in meters after that its a loop
    const double max_speed = 27.0;        // 60mph = 26.82m/s
    const double target_speed = 23.0;     // 50mph = 22.352m/s
    const double max_acceleration = 10.0; // 10m/s^2
    const double max_jerk = 50.0;         // 50/m/s^3
    const int frames = 50;               // 50 frames per second
    const int lanes = 3;
    const double lane_width = 4.0;

    inline Road() {}
    void updateData(json j, int index = 1);
    void readWayPointsFromFile(string filename);

    vector<double> distanceInFront(Vehicle &car, int lane);
    vector<double> distanceBehind(Vehicle &car, int lane);

    double closestVehicleAt(double s, double d, double time);
};

#endif // !ROAD_H