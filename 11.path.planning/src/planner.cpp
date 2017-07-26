#include "planner.h"

map<string, vector<string>> Planner::next_modes = {
    {"KL", {"KL", "LCL", "LCR", "MF"}},
    {"MF", {"MF", "KL", "LCL", "LCR"}},
    {"LCL", {"KL"}},
    {"LCR", {"KL"}},
};

string Planner::select_mode(Vehicle &car, Road &r)
{
    int current_time = 0;
    vector<pair<double, string>> next_modes_cost;
    const vector<string> possible_modes = Planner::next_modes[car.mode]; // a vector holding all the possible next modes

    for (const string next_mode : possible_modes)
    {
        double cost;
        if (next_mode.compare("LCL") == 0)
        {
            cost = costLaneChangeLeft(car, r);
        }
        else if (next_mode.compare("LCR") == 0)
        {
            cost = costLaneChangeRight(car, r);
        }
        else if (next_mode.compare("KL") == 0)
        {
            cost = costKeepLane(car, r);
        }
        else if (next_mode.compare("MF") == 0)
        {
            cost = costMatchFrontSpeed(car, r);
        }
        next_modes_cost.push_back(std::pair<double, string>(cost, next_mode));
    } // end of examining all possible next modes
    std::sort(next_modes_cost.begin(), next_modes_cost.end(), [](std::pair<double, string> &i, std::pair<double, string> &j) -> bool {
        return i.first < j.first;
    });
    string selected_mode = next_modes_cost[0].second;
    return selected_mode;
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

    if (frontDistance == car.carLength || behindDistance == car.carLength)
    {
        return maxCost;
    }
    double spaceNeeded = nearBuffer + car.carLength;
    double costForSpace = spaceNeeded / frontDistance + spaceNeeded / behindDistance - 2 * spaceNeeded;
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

    if (frontDistance == car.carLength || behindDistance == car.carLength)
    {
        return maxCost;
    }
    double spaceNeeded = nearBuffer + car.carLength;
    double costForSpace = spaceNeeded / frontDistance + spaceNeeded / behindDistance - 2 * spaceNeeded;
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
    if (frontDistance == car.carLength)
    {
        return maxCost;
    }
    double spaceNeeded = nearBuffer + car.carLength;
    double costForSpace = spaceNeeded / frontDistance;
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