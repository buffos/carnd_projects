#ifndef VEHICLE_H
#define VEHICLE_H

#include "json.hpp"
#include <vector>
#include <chrono>

using json = nlohmann::json;
using namespace std;

struct Vehicle
{
    double carLength = 5.0;
    double safetyRadius = 2.0; // safety radius
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
    string mode = "KL"; // keep lane

    Vehicle();
    Vehicle(double x, double y, double s, double d, double yaw, double speed);
    Vehicle(json j, int index = 1);
    Vehicle(const Vehicle &car);

    void updateData(json j, int index = 1);
    void readPreviousPath(json j, int index = 1);
    string createNextWebsocketMessage();

    int getLane();
    double getTargetD(int lane);
    // predictions
    bool collidesWith(Vehicle &other, double time = 0);
    pair<bool, int> willCollideWith(Vehicle &other, int timesteps, double dt);
    vector<double> getStateAt(double time);
};

#endif // !VEHICLE_H