/**
 * @file vehicle.cpp
 * @brief Implementation of the vehicle struct
 *
 *
 * @author  Kostas Oreopoulos
 */
#include "vehicle.h"
#include "tools.h"

// constructors
Vehicle::Vehicle() : x(0.0), y(0.0), s(0.0), d(0.0), yaw(0.0), speed(0.0), v_s(0.0), v_d(0.0), local_yaw(0.0) {}

Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double speed) {
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->speed = speed;
  this->yaw = yaw;
  this->local_yaw = 0.0;
  this->v_s = 0.0;
  this->v_d = 0.0;
}

Vehicle::Vehicle(const json j, const int index, const bool yawInDegrees) {
  updateData(j, index, yawInDegrees);
}

// update
void Vehicle::updateData(const json &j, const int index, const bool yawInDegrees) {
  x = j[index]["x"];
  y = j[index]["y"];
  s = j[index]["s"];
  d = j[index]["d"];
  // json gives the values in MPH. Only used by ego_car
  speed = MPH_to_MPS(j[index]["speed"]);
  acc = coords::accelerationFromCurve(previousCurve, 0, 10);
  yaw = (yawInDegrees) ? coords::deg2rad(j[index]["yaw"].get<double>()) :
        j[index]["yaw"].get<double>();
}

void Vehicle::updateLocalData(const double lane_yaw) {
  local_yaw = yaw - lane_yaw;
  v_s = speed * cos(local_yaw);
  v_d = speed * sin(local_yaw);
}

void Vehicle::readPreviousPath(const json &j, int index) {
  previousCurve.c_1 = j[index]["previous_path_x"].get<vector<double>>();
  previousCurve.c_2 = j[index]["previous_path_y"].get<vector<double>>();
  previousCurve.coordinateSystem = 1; // XY coordinates
}

void Vehicle::useRoadConfiguration(RoadConfiguration rcfg) {
  this->r = rcfg;
}

// getters
int Vehicle::getLane() const {
  if (d <= 0 or d >= r.lane_width * r.lanes) {
    return 0;
  }

  return static_cast<int>(floor(d / r.lane_width) + 1.0);
}

double Vehicle::getTargetD(int lane) const {
  double dd = (lane - 1.0) * r.lane_width + r.lane_width / 2.0;
  if (lane == 3) { dd -= 0.20; }
  // hack because the simulator for some reason report out of lane
  return dd;
}

vector<double> Vehicle::getStateAt(double time) const {
  double new_s = s + v_s * time;
  double new_d = d + v_d * time;
  return vector<double>{new_s, new_d, v_s, v_d};
}

int Vehicle::lag() {
  return constants::UPDATE_WHEN - previousCurve.size();
}

// collision check
bool Vehicle::collidesWith(Vehicle &other, double time) {
  double m_s, m_d, o_s, o_d;
  auto my_state = getStateAt(time);
  auto o_state = other.getStateAt(time);
  m_s = my_state[0];
  m_d = my_state[1];
  o_s = o_state[0];
  o_d = o_state[1];
  auto s_d = abs(m_s - o_s);
  auto d_d = abs(m_d - o_d);
  return (s_d < constants::SAFETY_DISTANCE && d_d < constants::SAFETY_WIDTH);
}

pair<bool, int> Vehicle::willCollideWith(Vehicle &other, int timesteps, double dt) {
  for (int i = 0; i <= timesteps; i++) {
    if (collidesWith(other, i * dt)) {
      return make_pair(true, i);
    }
  }
  return make_pair(false, -1);
}

// output
void Vehicle::printVehicle(ofstream &log) {

  log << "CAR STATE: " << endl;
  log << "X : " << x << " Y: " << y << " S : " << s << " d: " << d << endl;
  log << "YAW : " << yaw << " SPEED: " << speed << " ACC : " << acc << endl;
  log << "Current Mode: " << mode << endl;
}

void Vehicle::printVehicle() {

  cout << "CAR STATE: " << endl;
  cout << "X : " << x << " Y: " << y << " S : " << s << " d: " << d << endl;
  cout << "YAW : " << yaw << " SPEED: " << speed << " ACC : " << acc << endl;
  cout << "Current Mode: " << mode << endl << endl;
}