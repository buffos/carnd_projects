#ifndef TOOLS_H
#define TOOLS_H

#define _USE_MATH_DEFINES

#include <cmath>
#include <vector>
#include <math.h>
#include "various_structs.h"

using namespace std;

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
vector<double> real_s_distance(double s1, double s2, double trackLength);
};

#endif // !TOOLS_H