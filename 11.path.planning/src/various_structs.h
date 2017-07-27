#ifndef VARIOUS_STRUCTS_H
#define VARIOUS_STRUCTS_H

#include <vector>

using namespace std;


struct RoadConfiguration
{
    double max_s = 6945.554;        // in meters after that its a loop
    double max_speed = 27.0;        // 60mph = 26.82m/s
    double target_speed = 23.0;     // 50mph = 22.352m/s
    double max_acceleration = 10.0; // 10m/s^2
    double max_jerk = 50.0;         // 50/m/s^3
    int frames = 50;                // 50 frames per second
    int lanes = 3;
    double lane_width = 4.0;
};

struct StateGoal
{
	vector<double> start_s{ 0.0, 0.0, 0.0 };
	vector<double> end_s{ 0.0, 0.0, 0.0 };
	vector<double> start_d{ 0.0, 0.0, 0.0 };
	vector<double> end_d{ 0.0, 0.0, 0.0 };
};

struct Trajectory
{
	vector<double> s_trajectory; // polynomial coefficients
	vector<double> d_trajectory; // polynomial coefficients
	double duration;   // duration of trajectory
	double cost; // cost of the trajectory by the planner
};

struct WayPoint
{
	double x;
	double y;
	double s;
	double dx;
	double dy;

	inline WayPoint(double x, double y, double s, double dx, double dy) : x(x), y(y), s(s), dx(dx), dy(dy) {}
};

struct DiscreteCurve{
	vector<double> c_1;
	vector<double> c_2;
	int coordinateSystem;  // 1 = XY,  2 = Frenet
	double timestep; // the parameter-axis of those 2 parametric curves
};

#endif //! VARIOUS_STRUCTS_H