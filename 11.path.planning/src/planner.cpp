#include "planner.h"

map<string, vector<string>> Planner::next_modes = {
    {"KL", {"KL", "LCL", "LCR"}},
    {"LCL", {"KL"}},
    {"LCR", {"KL"}},
};

void Planner::update_mode(Vehicle &car, Road &r)
{
}

double Planner::costLaneChangeLeft(Vehicle &car, Road &r)
{
    int currentLane = car.getLane();

    if (currentLane == 1)
    {
        return maxCost;
    }

    vector<double> frontResults = r.distanceInFront(car, currentLane - 1);
    double frontDistance = frontResults[0];
    double frontSpeed = frontResults[1];
    double behindDistance = r.distanceBehind(car, currentLane - 1);

    if (frontDistance == 0 || behindDistance == 0)
    {
        return maxCost;
    }
    double cost = nearBuffer / frontDistance + nearBuffer / behindDistance - 2 * nearBuffer;
    // this is exactly zero when I have nearBuffer space both in front and behind the vehicle
    return cost;
}

double Planner::costLaneChangeRight(Vehicle &car, Road &r)
{
    int currentLane = car.getLane();

    if (currentLane == r.lanes)
    {
        return maxCost;
    }

    vector<double> frontResults = r.distanceInFront(car, currentLane + 1);
    double frontDistance = frontResults[0];
    double frontSpeed = frontResults[1];
    double behindDistance = r.distanceBehind(car, currentLane + 1);

    if (frontDistance == 0 || behindDistance == 0)
    {
        return maxCost;
    }
    double cost = nearBuffer / frontDistance + nearBuffer / behindDistance - 2 * nearBuffer;
    // this is exactly zero when I have nearBuffer space both in front and behind the vehicle
    return cost;
}

double Planner::costKeepLane(Vehicle &car, Road &r)
{
	return 0.0;
}