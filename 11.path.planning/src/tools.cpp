
#include "tools.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>



double coords::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

int coords::ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (unsigned int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = coords::distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int coords::NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = coords::ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = abs(theta - heading);

	if (angle > pi() / 4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> coords::getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = coords::NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
		prev_wp = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return { frenet_s,frenet_d };

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> coords::getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s*cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s*sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return { x,y };

}

void readWayPointsFromFile(string filename, vector<WayPoint> & wp) {
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
		wp.push_back(WayPoint(x, y, s, d_x, d_y));
	}
}// READ WAYPOINTS

// CAR HANDLING

CarData::CarData(double x, double y, double s, double d, double yaw, double speed) : x(x), y(y), s(s), d(d), yaw(yaw), speed(speed) {}
CarData::CarData(json j, int index) : CarData(j[index]["x"], j[index]["y"], j[index]["s"], j[index]["d"], j[index]["yaw"], j[index]["speed"]) {}

void CarData::readPreviousPath(json j, int index) {
	previous_x = j[index]["previous_path_x"].get<vector<double>>();
	previous_y = j[index]["previous_path_y"].get<vector<double>>();
	end_s = j[index]["end_path_s"];
	end_d = j[index]["end_path_d"];
}
void CarData::otherCarsFromSensorFusion(json j, int index) {
	auto sensor_fusion = j[index]["sensor_fusion"];
	otherCars = {};
	for (unsigned int i = 0; i < sensor_fusion.size(); i++) {
		int id = sensor_fusion[i][0];
		double x = sensor_fusion[i][1];
		double y = sensor_fusion[i][2];
		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double s = sensor_fusion[i][5];
		double d = sensor_fusion[i][6];
		double speed = sqrt(vx*vx + vy*vy);
		double yaw = atan2(vy, vx);
		CarData newCar(x, y, s, d, yaw, speed);
		otherCars.push_back(newCar);
	}
}

string CarData::createNextWebsocketMessage() {
	json msgJson;

	vector<double> next_x_vals;
	vector<double> next_y_vals;

	msgJson["next_x"] = next_x_vals;
	msgJson["next_y"] = next_y_vals;

	string msg = "42[\"control\"," + msgJson.dump() + "]";

	return msg;
}