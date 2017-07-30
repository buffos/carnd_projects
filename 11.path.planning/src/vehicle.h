#ifndef VEHICLE_H
#define VEHICLE_H

#include "json.hpp"
#include <vector>
#include <chrono>
#include "various_structs.h"

using json = nlohmann::json;
using namespace std;

// from mph to m/s
inline double MPH_to_MPS(double speed) { return speed * 0.44704; }

struct Vehicle
{
    double carLength = 5.0;
    double safetyRadius = 2.0; // safety radius
    RoadConfiguration r;
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    double acc = 0;
    chrono::steady_clock::time_point time;
    DiscreteCurve previousCurve;
    StateGoal currentGoal;
    double end_s = 0;
    double end_d = 0;
    bool init_clock = false;
    string mode = "KL"; // keep lane

    Vehicle();
    Vehicle(double x, double y, double s, double d, double yaw, double speed);
    Vehicle(json j, int index = 1, bool yawInDegrees = false); // if yawInDegrees it will be converted to rads
    Vehicle(const Vehicle &car);

    void updateData(json j, int index = 1, bool yawInDegrees = false);
    void readPreviousPath(json j, int index = 1);
    void useRoadConfiguration(RoadConfiguration rcfg);

    int getLane();
    double getTargetD(int lane);
    // predictions
    bool collidesWith(Vehicle &other, double time = 0);
    pair<bool, int> willCollideWith(Vehicle &other, int timesteps, double dt);
    vector<double> getStateAt(double time); // returns {lane, new_s, new_v, acc}
};

#endif // !VEHICLE_H