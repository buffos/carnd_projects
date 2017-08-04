#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "../include/vehicle.h"
#include "../include/road.h"
#include "../include/planner.h"
#include "trajectoryGenerator.h"
#include "../include/discreteCurves.h"


using namespace std;
using namespace testing;


class CurveChainTest : public ::testing::Test {
protected:
	virtual void SetUp() {
		RoadConfiguration rcfg;
		road.updateData(j, 1);
		road.readWayPointsFromFile(map_file_);
		car.updateData(j, 1);
		car.readPreviousPath(j, 1);
		car.useRoadConfiguration(rcfg);
	}

	// virtual void TearDown() {}
	Vehicle car;
	Road  road;
	Planner plan;
	TrajectoryGenerator tr_generator;
	CurveHandler curveHandler;

	string map_file_ = "C:/Users/buffo/Code/python/prj - selfDrivingCars/carnd-project/11.path.planning/data/highway_map.csv"; // Waypoint map to read from
	string s = "[\"telemetry\",{\"x\":909.48,\"y\":1128.67,\"yaw\":0,\"speed\":0,\"s\":124.8336,\"d\":6.164833,\"previous_path_x\":[],\"previous_path_y\":[],\"end_path_s\":0,\"end_path_d\":0,\"sensor_fusion\":[[0,826.3896,1128.832,23.23647,-0.1320468,41.80027,6.090794],[1,854.8988,1128.887,20.84263,-0.05619142,70.32576,5.989493],[2,1036.174,1155.92,17.97254,7.392407,253.5273,5.962598],[3,1031.829,1149.801,18.13678,7.502246,247.1916,9.984525],[4,775.8,1436.3,0,0,6737.538,-289.1815],[5,775.8,1441.7,0,0,6661.772,-291.7797],[6,762.1,1421.6,0,0,6738.859,-269.1307],[7,762.1,1425.2,0,0,6666.563,-270.8753],[8,762.1,1429,0,0,6663.543,-273.1828],[9,762.1,1432.9,0,0,6660.444,-275.5511],[10,762.1,1436.3,0,0,6638.274,-278.5977],[11,762.1,1441.7,0,0,6633.765,-281.5688]]}]";
	json j = json::parse(s);
};

//
//TEST_F(CurveChainTest, FirstTest) {
//	car.updateData(j, 1, true);											   
//	road.updateData(j);													   
//	car.readPreviousPath(j);											  
//	auto newMode = plan.select_mode(car, road);							   
//	auto newGoal = plan.realizePlan(newMode, car, road);				   
//	newGoal.printGoal();
//	auto trajectory = tr_generator.generateTrajectory(newGoal, car, road); 
//	trajectory.printTrajectory();
//	auto newCurve = std::move(curveHandler.createCurveFromCoefficientsInXY(trajectory, road.rcfg.frames, road.wpts));
//	auto mergedCurve = std::move(curveHandler.mergeCurves(newCurve, car.previousCurve));
//	mergedCurve.printCurve();
//
//}