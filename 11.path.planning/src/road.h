#ifndef ROAD_H
#define ROAD_H

#include "json.hpp"
#include <vector>
#include "vehicle.h"

using json = nlohmann::json;
using namespace std;

struct Road
{
    vector<Vehicle> cars{};
    const float max_speed = 27.0; // 60mph = 26.82m/s
    const float target_speed = 23.0; // 50mph = 22.352m/s
    const float max_acceleration = 10.0; // 10m/s^2
    const float max_jerk = 50.0; // 50/m/s^3
    const int frames = 50; // 50 frames per second
    const int lanes = 3;

    inline Road(){}
    void updateData(json j, int index = 1);
};

#endif // !ROAD_H