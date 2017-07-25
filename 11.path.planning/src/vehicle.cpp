#include "vehicle.h"
#include "spline.h"

Vehicle::Vehicle() : x(0), y(0), s(0), d(0), yaw(0), speed(0) {}

Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double speed) : x(x), y(y), s(s), d(d), yaw(yaw), speed(speed) {}

Vehicle::Vehicle(json j, int index) : Vehicle(j[index]["x"], j[index]["y"], j[index]["s"], j[index]["d"], j[index]["yaw"], j[index]["speed"]) {}

void Vehicle::updateData(json j, int index)
{
    if (!init_clock)
    {
        time = std::chrono::steady_clock::now();
        acc = 0;
        init_clock = true;
    }
    else
    {
        chrono::steady_clock::time_point newTime = std::chrono::steady_clock::now();
        double dt = 0.001 * chrono::duration_cast<std::chrono::microseconds>(newTime - time).count(); // in sec
        time = newTime;
        acc = (j[index]["speed"] - speed) / dt; // calculate acceleration based on dSpeed/dt
    }

    x = j[index]["x"];
    y = j[index]["y"];
    s = j[index]["s"];
    d = j[index]["d"];
    yaw = j[index]["yaw"];
    speed = j[index]["speed"];
}

void Vehicle::readPreviousPath(json j, int index)
{
    previous_x = j[index]["previous_path_x"].get<vector<double>>();
    previous_y = j[index]["previous_path_y"].get<vector<double>>();
    end_s = j[index]["end_path_s"];
    end_d = j[index]["end_path_d"];
}

string Vehicle::createNextWebsocketMessage()
{
    json msgJson;

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    msgJson["next_x"] = next_x_vals;
    msgJson["next_y"] = next_y_vals;

    string msg = "42[\"control\"," + msgJson.dump() + "]";

    return msg;
}

int Vehicle::getLane()
{
    if (d > 1.0 && d < 3.0)
    {
        return 1;
    }
    if (d > 5.0 && d < 7.0)
    {
        return 2;
    }
    if (d > 9.0 && d < 11.0)
    {
        return 3;
    }
    return 0;
}