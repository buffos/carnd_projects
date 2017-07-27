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

    if (frontDistance == 0 || behindDistance == 0)
    {
        return maxCost;
    }
    double spaceNeeded = nearBuffer;
    double costForSpace = spaceNeeded / frontDistance + spaceNeeded / behindDistance - 2 * spaceNeeded;
    // this is exactly zero when I have nearBuffer space both in front and behind the vehicle
    double costforSpeed = costSpeed(car, r.rcfg.target_speed, frontDistance, frontSpeed);
    return costForSpace + costforSpeed;
}

double Planner::costLaneChangeRight(Vehicle &car, Road &r)
{
    int currentLane = car.getLane();

    if (currentLane == r.rcfg.lanes)
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
    double spaceNeeded = nearBuffer;
    double costForSpace = spaceNeeded / frontDistance + spaceNeeded / behindDistance - 2 * spaceNeeded;
    // this is exactly zero when I have nearBuffer space both in front and behind the vehicle
    double costforSpeed = costSpeed(car, r.rcfg.target_speed, frontDistance, frontSpeed);
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
    double spaceNeeded = nearBuffer;
    double costForSpace = spaceNeeded / frontDistance;
    double costforSpeed = costSpeed(car, r.rcfg.target_speed, frontDistance, frontSpeed);
    return costForSpace + costforSpeed;
}

double Planner::costMatchFrontSpeed(Vehicle &car, Road &r)
{
    int currentLane = car.getLane();
    double slowDownCost = 0.0;

    vector<double> frontResults = r.distanceInFront(car, currentLane);
    double frontDistance = frontResults[0];
    double frontSpeed = frontResults[1];

    if (frontSpeed < r.rcfg.target_speed)
    { // the front car is moving slower than my target speed
        slowDownCost = 100 * (r.rcfg.target_speed - frontSpeed);
    }
    return slowDownCost;
}

double Planner::costSpeed(Vehicle &car, double desiredSpeed, double freeRoadAhead, double speedCarInFront)
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

StateGoal Planner::realizePlan(string mode, Vehicle &car, Road &r)
{
    if (mode.compare("KL") == 0)
    {
        return realizeKeepLane(car, r);
    }
    else if (mode.compare("MF") == 0)
    {
        return realizeMatchFront(car, r);
    }
    else if (mode.compare("LCL") == 0)
    {
        return realizeChangeLeft(car, r);
    }
    else if (mode.compare("LCR") == 0)
    {
        return realizeChangeRight(car, r);
    }
}

StateGoal Planner::realizeKeepLane(Vehicle &car, Road &r)
{
    StateGoal goal;
    double newAcceleration = (r.rcfg.target_speed - car.speed) / planDuration;
    goal.start_s = {car.s, car.speed, car.acc};
    goal.start_d = {car.d, 0.0, 0.0};

    goal.end_d = {car.getTargetD(car.getLane()), 0.0, 0.0}; // maybe the car is not centered so center it
    goal.end_s = {
        car.s + 0.5 * (r.rcfg.target_speed + car.speed) * planDuration, // just do algebra on vT + 0.5(targetV - v)*T^2/T
		r.rcfg.target_speed,
        newAcceleration};
    goal.end_d = {car.d, 0.0, 0.0};
    // no lateral movement so d is the default zero everywhere
    return goal;
}

StateGoal Planner::realizeMatchFront(Vehicle &car, Road &r)
{
    StateGoal goal;

    vector<double> frontResults = r.distanceInFront(car, car.getLane());
    double frontDistance = frontResults[0];
    double frontSpeed = frontResults[1];

    double newAcceleration = (frontSpeed - car.speed) / planDuration;
    goal.start_s = {car.s, car.speed, car.acc};
    goal.start_d = {car.d, 0.0, 0.0};

    goal.end_d = {car.getTargetD(car.getLane()), 0.0, 0.0};
    goal.end_s = {
        car.s + 0.5 * (frontSpeed + car.speed) * planDuration,
        frontSpeed,
        newAcceleration};
    return goal;
}

StateGoal Planner::realizeChangeLeft(Vehicle &car, Road &r)
{
    // keep speed but shift left
    StateGoal goal;
    goal.start_s = {car.s, car.speed, car.acc};
    goal.start_d = {car.d, 0.0, 0.0};

    goal.end_s = {
        car.s + car.speed * planDuration,
        car.speed,
        0};
    double delta_d = car.d - r.rcfg.lane_width;
    double delta_v = delta_d / planDuration;
    double delta_g = delta_v / planDuration;

    goal.end_d = {delta_d, delta_v, delta_g};
    return goal;
}

StateGoal Planner::realizeChangeRight(Vehicle &car, Road &r)
{
    StateGoal goal;
    goal.start_s = {car.s, car.speed, car.acc};
    goal.start_d = {car.d, 0.0, 0.0};

    goal.end_s = {
        car.s + car.speed * planDuration,
        car.speed,
        0};
    double delta_d = car.d + r.rcfg.lane_width;
    double delta_v = delta_d / planDuration;
    double delta_g = delta_v / planDuration;

    goal.end_d = {delta_d, delta_v, delta_g};
    return goal;
}