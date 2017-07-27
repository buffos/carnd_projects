#include "vehicle.h"
#include "spline.h"

Vehicle::Vehicle() : x(0), y(0), s(0), d(0), yaw(0), speed(0) {}

Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double speed) : x(x), y(y), s(s), d(d), yaw(yaw), speed(speed) {}

Vehicle::Vehicle(json j, int index) : Vehicle(j[index]["x"], j[index]["y"], j[index]["s"], j[index]["d"], j[index]["yaw"], j[index]["speed"]) {}

Vehicle::Vehicle(const Vehicle &car)
{
    x = car.x;
    y = car.y;
    s = car.s;
    d = car.d;
    yaw = car.yaw;
    speed = car.speed;
    acc = car.acc;
    time = car.time;
    previousCurve = car.previousCurve;
    end_s = car.end_s;
    end_d = car.end_d;
    init_clock = true;
}

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
    previousCurve.c_1 = j[index]["previous_path_x"].get<vector<double>>();
    previousCurve.c_2 = j[index]["previous_path_y"].get<vector<double>>();
    previousCurve.coordinateSystem = 1; // XY coordinates
    end_s = j[index]["end_path_s"];
    end_d = j[index]["end_path_d"];
}

void Vehicle::useRoadConfiguration(RoadConfiguration rcfg)
{
    this->r = rcfg;
}

int Vehicle::getLane()
{
    if (d < 0 or d > r.lane_width * r.lanes)
    {
        return 0;
    }
    return static_cast<int>(floor(d / r.lane_width) + 1);
}

double Vehicle::getTargetD(int lane)
{
    return (lane - 1) * r.lane_width + r.lane_width / 2;
}

bool Vehicle::collidesWith(Vehicle &other, double time)
{
    auto my_state = getStateAt(time);
    auto other_state = other.getStateAt(time);
    double myLane = my_state[0];
    double otherLane = other_state[0];
    double myS = my_state[1];
    double otherS = other_state[1];
    return (myLane == otherLane && abs(myS - otherS) < carLength);
    return true;
}

pair<bool, int> Vehicle::willCollideWith(Vehicle &other, int timesteps, double dt)
{
    for (int i = 0; i <= timesteps; i++)
    {
        if (collidesWith(other, i * dt))
        {
            return pair<bool, int>(true, i);
        }
    }
    return pair<bool, int>(false, -1);
}

vector<double> Vehicle::getStateAt(double time)
{
    double new_s = s + speed * time + acc * time * time / 2;
    double new_v = speed + acc * time;
    double lane = (double)getLane();
    return vector<double>{lane, new_s, new_v, acc};
}