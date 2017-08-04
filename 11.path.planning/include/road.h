#ifndef ROAD_H
#define ROAD_H

#include "../src/json.hpp"
#include <vector>
#include "vehicle.h"
#include "tools.h"
#include "various_structs.h"
#include "spline.h"

using json = nlohmann::json;
using namespace std;

struct Road {
  vector<Vehicle> cars;
  vector<WayPoint> wpts;
  const RoadConfiguration rcfg;
  Splines track_spline;

  inline Road() {}
  void updateData(const json j, int index = 1);
  void readWayPointsFromFile(string filename);

  vector<double> distanceInFront(const Vehicle &car, int lane) const;
  vector<double> distanceBehind(const Vehicle &car, int lane) const ;

  double closestVehicleAt(double s, double d, double time);
  void createTrackSpline();
  vector<double> toXY(double s, double d) const;

  vector<double> estimateCurvatureFactor(double s, double d,
                                         const int evaluationPoints,
                                         double scanningDistance) const ;
};

#endif // !ROAD_H