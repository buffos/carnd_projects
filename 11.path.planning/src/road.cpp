#include "road.h"

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

double Road::distanceInFront(Vehicle &car, int lane)
{
    // this function will look ahead in the given lane and find the distance to the closest car in front
    double max_distance = 100.0; // check up to 100 meters
    double current_s = 9999999.0;
    for (auto&& other_car : cars)
    {
        if (car.getLane() == other_car.getLane())
        {
            if ((car.s > max_s - max_distance) && (car.s < other_car.s) && (other_car.s < max_s))
            {
                // both cars are right before the end of the lap , so i take the difference
                current_s = min(current_s, other_car.s - car.s);
            }
            else if ((car.s > max_s - max_distance) && (other_car.s < max_distance))
            {
                // the car is just before the end and the other car has just crossed the end
                // so the distance is what is left to the end (max_s - car.s) and what the other car has traveled (other_car.s)
                double distance = max_s - car.s + other_car.s;
                current_s = min(current_s, distance);
            }
            else if (car.s <= max_s - max_distance && car.s < other_car.s)
            {
                // the cars are in a zone that can be compared
                current_s = min(current_s, other_car.s - car.s);
            }
        }
    }
    return current_s;
}

double Road::distanceBehind(Vehicle &car, int lane)
{
    // this function will look ahead in the given lane and find the distance to the closest car behind
    double max_distance = 100.0; // check up to 100 meters
    double current_s = 9999999.0;
    for (auto&& other_car : cars)
    {
        if (car.getLane() == other_car.getLane())
        {
            if ((other_car.s > max_s - max_distance) && (other_car.s < car.s) && (car.s < max_s))
            {
                // both cars are right before the end of the lap , and my car is in front, so i take the difference
                current_s = min(current_s, car.s - other_car.s);
            }
            else if ((other_car.s > max_s - max_distance) && (car.s < max_distance))
            {
                // the other car is just before the end and the my car has just crossed the end
                // so the distance is what is left to the end (max_s - other_car.s) and what my car has traveled (car.s)
                double distance = max_s - other_car.s + car.s;
                current_s = min(current_s, distance);
            }
            else if (other_car.s <= max_s - max_distance && other_car.s < car.s)
            {
                // the cars are in a zone that can be compared and other car is behind
                current_s = min(current_s, car.s - other_car.s);
            }
        }
    }
    return current_s;
}