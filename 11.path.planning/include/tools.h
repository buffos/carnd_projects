/**
 * @file tools.h
 * @brief Helper functions contained in coords namespace
 *
 *
 * @author  Kostas Oreopoulos
 */
#ifndef TOOLS_H
#define TOOLS_H

#define M_PI 3.141592654

#include <cmath>
#include <vector>
#include <math.h>
#include "various_structs.h"
#include "constants.h"

using namespace std;

double logistic(double x);

namespace coords {
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * coords::pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2);

double accelerationFromCurve(const DiscreteCurve &curve,
                             int fromPoint, int toPoint);

int ClosestWaypoint(double x, double y, const vector<WayPoint> &wp);

int NextWaypoint(double x, double y, double theta, const vector<WayPoint> &wp);

vector<double> getFrenet(double x, double y, double theta,
                         const vector<WayPoint> &wp);
vector<double> getXY(double s, double d, const vector<WayPoint> &wp);

vector<int> getLocalWayPointIndexes(int index, int back,
                                    int front, int wp_size);
vector<double> evaluateSplineAtS(double s, double d,
                                 const Splines &sp, double trackLength);
bool isPointInSpline(double s, const Splines &sp);
Splines createLocalSplines(double s, int back, int front,
                           const vector<WayPoint> &wp, double trackLength);
};

#endif // !TOOLS_H