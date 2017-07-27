#ifndef TOOLS_H
#define TOOLS_H
#define _USE_MATH_DEFINES

#include <vector>

using namespace std;

struct WayPoint
{
	double x;
	double y;
	double s;
	double dx;
	double dy;

	inline WayPoint(double x, double y, double s, double dx, double dy) : x(x), y(y), s(s), dx(dx), dy(dy) {}
};

void readWayPointsFromFile(string filename, vector<WayPoint> &wp);

namespace coords
{
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * coords::pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, vector<WayPoint> &wp);
int NextWaypoint(double x, double y, double theta, vector<WayPoint> &wp);
vector<double> getFrenet(double x, double y, double theta, vector<WayPoint> &wp);
vector<double> getXY(double s, double d, vector<WayPoint> &wp);
};


#endif // !TOOLS_H