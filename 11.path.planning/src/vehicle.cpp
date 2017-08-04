#include "vehicle.h"
#include "tools.h"

Vehicle::Vehicle() : x(0.0), y(0.0), s(0.0), d(0.0), yaw(0.0), speed(0.0) {}

Vehicle::Vehicle(double x, double y, double s, double d, double yaw,
                 double speed) : x(x), y(y), s(s), d(d),
                                 yaw(yaw), speed(speed) {}

Vehicle::Vehicle(const json j, const int index, const bool yawInDegrees) {
  updateData(j, index, yawInDegrees);
}

//Vehicle::Vehicle(const Vehicle &car) {
//  x = car.x;
//  y = car.y;
//  s = car.s;
//  d = car.d;
//  yaw = car.yaw;
//  speed = car.speed;
//  acc = car.acc;
//  time = car.time;
//  previousCurve = car.previousCurve;
//  end_s = car.end_s;
//  end_d = car.end_d;
//  init_clock = true;
//}

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

void Vehicle::readPreviousPath(const json &j, int index) {
  previousCurve.c_1 = j[index]["previous_path_x"].get<vector<double>>();
  previousCurve.c_2 = j[index]["previous_path_y"].get<vector<double>>();
  previousCurve.coordinateSystem = 1; // XY coordinates
  end_s = j[index]["end_path_s"];
  end_d = j[index]["end_path_d"];
}


void Vehicle::useRoadConfiguration(RoadConfiguration rcfg) {
  this->r = rcfg;
}


int Vehicle::getLane() const {
  //if (d <= 0 or d >= r.lane_width * r.lanes) {
  //  return 0;
  //}

  //return static_cast<int>(floor(d / r.lane_width) + 1.0);
	int lane = 0;
	if (d > 0.0 && d < 4.0) {
		lane = 1;
	}
	else if (d > 4.0 && d < 8.0) {
		lane = 2;
	}
	else if (d > 8.0 && d < 12.0) {
		lane = 3;
	}
	return lane;
}


double Vehicle::getTargetD(int lane) const {
  return (lane - 1.0) * r.lane_width + r.lane_width / 2.0;
}


bool Vehicle::collidesWith(Vehicle &other, double time) {
  auto my_state = getStateAt(time);
  auto other_state = other.getStateAt(time);
  double myLane = my_state[0];
  double otherLane = other_state[0];
  double myS = my_state[1];
  double otherS = other_state[1];
  return (myLane == otherLane && abs(myS - otherS) < carLength);
}


pair<bool, int> Vehicle::willCollideWith(Vehicle &other,
                                         int timesteps,
                                         double dt) {
  for (int i = 0; i <= timesteps; i++) {
    if (collidesWith(other, i * dt)) {
      return pair<bool, int>{true, i};
    }
  }
  return pair<bool, int>{false, -1};
}


vector<double> Vehicle::getStateAt(double time) {
  double new_s = s + speed * time;
  double new_v = speed;
  return vector<double>{new_s, new_v};
}


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