#ifndef VEHICLE_H
#define VEHICLE_H

#include "json.hpp"
#include <vector>
#include <chrono>

using json = nlohmann::json;
using namespace std;

struct Vehicle
{
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    double acc = 0;
    chrono::steady_clock::time_point time;
    vector<double> previous_x{};
    vector<double> previous_y{};
    double end_s = 0;
    double end_d = 0;
    bool init_clock = false;

    Vehicle();
    Vehicle(double x, double y, double s, double d, double yaw, double speed);
    Vehicle(json j, int index = 1);
    void updateData(json j, int index = 1);
    void readPreviousPath(json j, int index = 1);
    string createNextWebsocketMessage();

    int getLane();
};

#endif // !VEHICLE_H