#include "road.h"

void Road::updateData(json j, int index)
{
    auto sensor_fusion = j[index]["sensor_fusion"];
    cars = {};
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