/**
 * @file road.cpp
 * @brief Implementation of the Road struct
 *
 * @author  Kostas Oreopoulos
 */
#include "road.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

void Road::updateData(const json j, int index) {
  auto sensor_fusion = j[index]["sensor_fusion"];
  cars.clear();
  for (unsigned int i = 0; i < sensor_fusion.size(); i++) {
    int id = sensor_fusion[i][0];
    double x = sensor_fusion[i][1];
    double y = sensor_fusion[i][2];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double s = sensor_fusion[i][5];
    double d = sensor_fusion[i][6];
    double v = sqrt(vx * vx + vy * vy);
    double yaw = atan2(vy, vx);
    double local_yaw = yaw - orientation(s);
    Vehicle newCar(x, y, s, d, yaw, v);
    newCar.local_yaw = local_yaw;
    newCar.v_s = v * cos(local_yaw);
    newCar.v_d = v * sin(local_yaw);
    cars.push_back(newCar);
  }
}

void Road::readWayPointsFromFile(string filename) {
  ifstream in_map_(filename.c_str(), ifstream::in);
  string line;

  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    wpts.push_back(WayPoint(x, y, s, d_x, d_y));
  }
}

vector<int> Road::adjacentLanes(int lane) const {
  if (lane > 1 && lane < rcfg.lanes) {
    return {lane - 1, lane + 1};
  }
  if (lane == 1) {
    return {lane + 1};
  }
  return {lane - 1};
}

vector<double> Road::distanceInFront(const Vehicle &car, int lane) const {
  // this function will look ahead in the given lane
  // and find the distance to the closest car in front
  double d_closest = std::numeric_limits<double>::max();
  double v_closest = std::numeric_limits<double>::max();
  int i_closest = 0;

  double s = car.s;
  double S_MAX = constants::TRACKLENGTH;

  for (unsigned int i = 0; i < cars.size(); i++) {
    if (lane != cars[i].getLane()) {
      continue;
    }
    auto s_i = cars[i].s;
    double d = (s <= s_i ? s_i - s : s_i + S_MAX - s);
    if (d < d_closest) {
      d_closest = d;
      i_closest = i;
      v_closest = cars[i].speed;
    }
  }

  return vector<double>{d_closest, v_closest, (double) i_closest};
}

vector<double> Road::distanceBehind(const Vehicle &car, int lane) const {
  // this function will look behind in the given lane
  // and find the distance to the closest car in front
  double d_closest = std::numeric_limits<double>::max();
  double v_closest = std::numeric_limits<double>::max();
  int i_closest = 0;

  double s = car.s;
  double S_MAX = constants::TRACKLENGTH;

  for (unsigned int i = 0; i < cars.size(); i++) {
    if (lane != cars[i].getLane()) {
      continue;
    }
    auto s_i = cars[i].s;
    double d = (s_i <= s ? s - s_i : s + S_MAX - s_i);
	d = (abs(s_i - s) < constants::CAR_LENGTH) ? 0 : d;
    if (d < d_closest) {
      d_closest = d;
      i_closest = i;
	  v_closest = cars[i].speed;
    }
  }

  return vector<double>{d_closest, v_closest, (double) i_closest};
}

double Road::closestVehicleAt(double s, double d, double time) {

// this function will look behind in the given lane
  // and find the distance to the closest car in front
  double d_closest = std::numeric_limits<double>::max();
  int i_closest = 0;

  for (unsigned int i = 0; i < cars.size(); i++) {
    double o_s, o_d;
    auto c_state = cars[i].getStateAt(time);
    o_s = c_state[0];
    o_d = c_state[1];
    double d2 = sqrt(pow(o_s - s, 2) + pow(o_d - d, 2));
    if (d2 < d_closest) {
      d_closest = d2;
      i_closest = i;
    }
  }
  return d_closest;
}

void Road::createTrackSpline() {
  vector<double> x;
  vector<double> y;
  vector<double> ss;
  vector<double> dx;
  vector<double> dy;

  for (auto const &point : wpts) {
    x.push_back(point.x);
    y.push_back(point.y);
    ss.push_back(point.s);
    dx.push_back(point.dx);
    dy.push_back(point.dy);
  }
  // close the loop
  x.push_back(wpts[0].x);
  y.push_back(wpts[0].y);
  ss.push_back(constants::TRACKLENGTH);
  dx.push_back(wpts[0].dx);
  dy.push_back(wpts[0].dy);

  track_spline.x.set_points(ss, x);
  track_spline.y.set_points(ss, y);
  track_spline.dx.set_points(ss, dx);
  track_spline.dy.set_points(ss, dy);
  track_spline.start_s = 0;
  track_spline.end_s = constants::TRACKLENGTH;
}

vector<double> Road::toXY(const double s, const double d) const {
  double real_s = fmod(s, constants::TRACKLENGTH);
  double x = track_spline.x(real_s);
  double y = track_spline.y(real_s);
  double dx = track_spline.dx(real_s);
  double dy = track_spline.dy(real_s);

  x += dx * d;
  y += dy * d;

  return {x, y};
}

double Road::orientation(double s) {
  auto xy_a = toXY(s, 0);
  auto xy_b = toXY(s + 1.0, 0);
  auto dx = xy_b[0] - xy_a[0];
  auto dy = xy_b[1] - xy_a[1];
  return atan2(dy, dx);
}
vector<double> Road::curvatureFactor(const double s, const double d, const int evaluationPoints,
                                     const double scanningDistance) const {
  // take a piece of 100 meters road and split it into 4 points
  // add the XY distance and see how bigger its from s distance, which is 100
  const auto points = evaluationPoints;
  const auto distPerPoint = scanningDistance / (points - 1);
  auto xyDistance = 0.0;
  for (int i = 0; i < points - 1; i++) {
    auto current_s = fmod(s + i * distPerPoint, constants::TRACKLENGTH);
    auto next_s = fmod(s + (i + 1) * distPerPoint, constants::TRACKLENGTH);
    auto current_xy = toXY(current_s, d);
    auto next_xy = toXY(next_s, d);
    auto newDistance = coords::distance(current_xy[0], current_xy[1],
                                        next_xy[0], next_xy[1]);
    xyDistance += newDistance;
  }
  auto factor = xyDistance / scanningDistance;
  return vector<double>{factor, xyDistance};
}
