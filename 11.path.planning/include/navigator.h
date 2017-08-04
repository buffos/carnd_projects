#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "json.hpp"
#include "tools.h"
#include "vehicle.h"
#include "road.h"
#include "planner.h"
#include "trajectoryGenerator.h"
#include "discreteCurves.h"
#include "constants.h"


struct Navigator{
	string map_file = R"(C:/Users/buffo/Code/python/prj - selfDrivingCars/carnd-project/11.path.planning/data/highway_map.csv)";
	string log_file = R"(I:\logger.txt)";
	Vehicle car;
	Road road;
	Planner plan;
	TrajectoryGenerator tr_generator;
	CurveHandler curveHandler;
	ofstream logger;

	Navigator();
	Navigator(const string &filename);
	~Navigator();


	string operator () (const nlohmann::json &json);

};

#endif