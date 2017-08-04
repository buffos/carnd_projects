#ifndef VEHICLE_H
#define VEHICLE_H

#include "../src/json.hpp"
#include <fstream>
#include <vector>
#include <chrono>
#include "various_structs.h"

using json = nlohmann::json;
using namespace std;

// from mph to m/s
inline double MPH_to_MPS(double speed) { return speed * 0.44704; }

struct Vehicle {
  double carLength = 5.0;
  double safetyRadius = 2.0; // safety radius
  RoadConfiguration r;
  double x , y, s, d, yaw, speed;
  double acc = 0.0;
  double end_s = 0.0;
  double end_d = 0.0;
  DiscreteCurve previousCurve;
  StateGoal currentGoal;

  string mode = "OFF"; // OFF in the beginning

  Vehicle();
  Vehicle(double x, double y, double s, double d, double yaw, double speed);
  // Vehicle(const Vehicle &car); // if yawInDegrees it will be converted to
  // rads

  explicit Vehicle(const json j, int index = 1, bool yawInDegrees = false);

  void updateData(const json &j, int index = 1, bool yawInDegrees = false);
  void readPreviousPath(const json &j, int index = 1);
  void useRoadConfiguration(RoadConfiguration rcfg);
  void printVehicle(ofstream &log);
  void printVehicle();

  int getLane() const ;
  double getTargetD(int lane) const ;

  // predictions
  bool collidesWith(Vehicle &other, double time = 0);
  pair<bool, int> willCollideWith(Vehicle &other, int timesteps, double dt);
  vector<double> getStateAt(double time); // returns {lane, new_s, new_v, acc}
};

#endif // !VEHICLE_H