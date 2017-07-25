#ifndef TOOLS_H
#define TOOLS_H
#define _USE_MATH_DEFINES

#include "json.hpp"
#include <vector>

using json = nlohmann::json;
using namespace std;

namespace coords {
	constexpr double pi() { return M_PI; }
	inline double deg2rad(double x) { return x * coords::pi() / 180; }
	inline double rad2deg(double x) { return x * 180 / pi(); }
	double distance(double x1, double y1, double x2, double y2);
	int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
	int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
	vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
	vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

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

struct CarData {
	double x;
	double y;
	double s;
	double d;
	double yaw;
	double speed;
	vector<double> previous_x{};
	vector<double> previous_y{};
	vector<CarData> otherCars{};
	double end_s = 0;
	double end_d = 0;

	CarData(double x, double y, double s, double d, double yaw, double speed);
	CarData(json j, int index=1);
	void readPreviousPath(json j, int index=1);
	void otherCarsFromSensorFusion(json j, int index = 1);
};

void readWayPointsFromFile(string filename, vector<WayPoint> &wp);

#endif // !TOOLS_H