#include "road.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

void Road::updateData(json j, int index)
{
    auto sensor_fusion = j[index]["sensor_fusion"];
    for (unsigned int i = 0; i < sensor_fusion.size(); i++)
    {
        int id = sensor_fusion[i][0];
        double x = sensor_fusion[i][1];
        double y = sensor_fusion[i][2];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double s = sensor_fusion[i][5];
        double d = sensor_fusion[i][6];
        double speed = sqrt(vx * vx + vy * vy);
        double yaw = atan2(vy, vx);
        Vehicle newCar(x, y, s, d, yaw, speed);
        cars.push_back(newCar);
    }
}

void Road::readWayPointsFromFile(string filename)
{
    ifstream in_map_(filename.c_str(), ifstream::in);
    string line;

    while (getline(in_map_, line))
    {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        wpts.push_back(WayPoint(x, y, s, d_x, d_y));
    }
}

vector<double> Road::distanceInFront(Vehicle &car, int lane)
{
    // this function will look ahead in the given lane and find the distance to the closest car in front
    double current_s = 9999999.0;
    double speed_carInFront = 1000000.0;
    double other_car_length = car.carLength;
    for (auto &&other_car : cars)
    {
        if (car.getLane() == other_car.getLane())
        {
            auto car_distance = coords::real_s_distance(car.s, other_car.s, max_s);

            if (car_distance[0] < current_s && car_distance[1] == 2)
            { // car_distance[1] == 2 means other car (argument 2) in front
                current_s = car_distance[0];
                speed_carInFront = other_car.speed;
                other_car_length = other_car.carLength;
            }
        }
    }
    // s treats car like points so add the physical half lenghts
    current_s -= (car.carLength + other_car_length) * 0.5;
    return vector<double>{current_s, speed_carInFront};
}

vector<double> Road::distanceBehind(Vehicle &car, int lane)
{
    // this function will look ahead in the given lane and find the distance to the closest car behind
    double max_distance = 100.0; // check up to 100 meters
    double current_s = 9999999.0;
    double speed_carBehind = 1000000.0;
    double other_car_length = car.carLength;
    for (auto &other_car : cars)
    {
        if (car.getLane() == other_car.getLane())
        {
            auto car_distance = coords::real_s_distance(car.s, other_car.s, max_s);

            if (car_distance[0] < current_s && car_distance[1] == 1)
            { // car_distance[1] == 1 means my car (argument 1) in front. so other_car is behind
                current_s = car_distance[0];
                speed_carBehind = other_car.speed;
                other_car_length = other_car.carLength;
            }
        }
    }
    // s treats car like points so add the physical half lenghts
    current_s -= (car.carLength + other_car_length) * 0.5;
    return vector<double>{current_s, speed_carBehind};
}

double Road::closestVehicleAt(double s, double d, double time)
{
    double closest = 99999999.0;
    for (auto &other_car : cars)
    {
        auto state = other_car.getStateAt(time);
        double s_distance = coords::real_s_distance(state[0], s, max_s)[0]; // the first entry is the distance
        double distance = sqrt(s_distance * s_distance + (state[1] - d) * (state[1] - d));
        if (distance < closest)
        {
            closest = distance;
        }
    }
    return closest;
}
