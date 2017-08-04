#ifndef ROAD_H
#define ROAD_H

#include "json.hpp"
#include <vector>
#include "vehicle.h"
#include "tools.h"
#include "various_structs.h"
#include "spline.h"

using json = nlohmann::json;
using namespace std;

struct Road {
  const RoadConfiguration rcfg;
  vector<Vehicle> cars;
  vector<WayPoint> wpts;
  Splines track_spline;

  inline Road() {}
  void updateData(const json j, int index = 1);
  void readWayPointsFromFile(string filename);
  void createTrackSpline();

  vector<double> distanceInFront(const Vehicle &car, int lane) const;
  vector<double> distanceBehind(const Vehicle &car, int lane) const;
  double closestVehicleAt(double s, double d, double time);

  vector<double> toXY(double s, double d) const;
  vector<double> curvatureFactor(double s, double d, const int evaluationPoints, double scanningDistance) const;
  double orientation(double s);
};

#endif // !ROAD_H