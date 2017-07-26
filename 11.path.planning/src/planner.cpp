#include "planner.h"

map<string, vector<string>> Planner::next_modes = {
    {"KL", {"KL", "LCL", "LCR", "MF"}},
    {"MF", {"MF", "KL", "LCL", "LCR"}},
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
    vector<double> behindResults = r.distanceBehind(car, currentLane - 1);
    double frontDistance = frontResults[0];
    double frontSpeed = frontResults[1];
    double behindDistance = behindResults[0];
    double behindSpeed = behindResults[1];

    if (frontDistance == 0 || behindDistance == 0)
    {
        return maxCost;
    }
    double costForSpace = nearBuffer / frontDistance + nearBuffer / behindDistance - 2 * nearBuffer;
    // this is exactly zero when I have nearBuffer space both in front and behind the vehicle
    double costforSpeed = costSpeed(r.target_speed, frontDistance, frontSpeed);
    return costForSpace + costforSpeed;
}

double Planner::costLaneChangeRight(Vehicle &car, Road &r)
{
    int currentLane = car.getLane();

    if (currentLane == r.lanes)
    {
        return maxCost;
    }

    vector<double> frontResults = r.distanceInFront(car, currentLane + 1);
    vector<double> behindResults = r.distanceBehind(car, currentLane + 1);
    double frontDistance = frontResults[0];
    double frontSpeed = frontResults[1];
    double behindDistance = behindResults[0];
    double behindSpeed = behindResults[1];

    if (frontDistance == 0 || behindDistance == 0)
    {
        return maxCost;
    }
    double costForSpace = nearBuffer / frontDistance + nearBuffer / behindDistance - 2 * nearBuffer;
    // this is exactly zero when I have nearBuffer space both in front and behind the vehicle
    double costforSpeed = costSpeed(r.target_speed, frontDistance, frontSpeed);
    return costForSpace + costforSpeed;
}

double Planner::costKeepLane(Vehicle &car, Road &r)
{
    int currentLane = car.getLane();
    vector<double> frontResults = r.distanceInFront(car, currentLane);
    double frontDistance = frontResults[0];
    double frontSpeed = frontResults[1];
    if (frontDistance == 0)
    {
        return maxCost;
    }
    double costForSpace = nearBuffer / frontDistance;
    double costforSpeed = costSpeed(r.target_speed, frontDistance, frontSpeed);
    return costForSpace + costforSpeed;
}

double Planner::costMatchFrontSpeed(Vehicle &car, Road &r)
{
    int currentLane = car.getLane();
    double slowDownCost = 0.0;

    vector<double> frontResults = r.distanceInFront(car, currentLane);
    double frontDistance = frontResults[0];
    double frontSpeed = frontResults[1];

    if (frontSpeed < r.target_speed)
    { // the front car is moving slower than my target speed
        slowDownCost = 100 * (r.target_speed - frontSpeed);
    }
    return slowDownCost;
}

double Planner::costSpeed(double desiredSpeed, double freeRoadAhead, double speedCarInFront)
{
    double relativeSpeed = desiredSpeed - speedCarInFront;
    if (relativeSpeed <= 0)
    {
        return 0.0; // no cost. the car in front is going very fast or no car in front
    }
    else
    {
        double timeToReachFrontCar = (freeRoadAhead - nearBuffer) / relativeSpeed;
        return 100 * timeToReachFrontCar;
    }
}