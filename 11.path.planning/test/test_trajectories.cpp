#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "vehicle.h"
#include "road.h"
#include "planner.h"
#include "trajectoryGenerator.h"
#include "discreteCurves.h"

using namespace std;
using namespace testing;


class TrajectoryTest : public ::testing::Test {
protected:
	virtual void SetUp() {
		RoadConfiguration rcfg;
		road.updateData(j, 1);
		road.readWayPointsFromFile(map_file_);
		v1_.updateData(j, 1);
		v1_.readPreviousPath(j, 1);
		v1_.useRoadConfiguration(rcfg);
	}

	// virtual void TearDown() {}
	Vehicle v1_;
	Vehicle v2_;
	Road  road;
	Planner plan;
	TrajectoryGenerator tr;

	string map_file_ = "C:/Users/buffo/Code/python/prj - selfDrivingCars/carnd-project/11.path.planning/data/highway_map.csv"; // Waypoint map to read from
	string s = "[\"telemetry\",{\"x\":909.48,\"y\":1128.67,\"yaw\":0,\"speed\":0,\"s\":124.8336,\"d\":6.164833,\"previous_path_x\":[],\"previous_path_y\":[],\"end_path_s\":0,\"end_path_d\":0,\"sensor_fusion\":[[0,826.3896,1128.832,23.23647,-0.1320468,41.80027,6.090794],[1,854.8988,1128.887,20.84263,-0.05619142,70.32576,5.989493],[2,1036.174,1155.92,17.97254,7.392407,253.5273,5.962598],[3,1031.829,1149.801,18.13678,7.502246,247.1916,9.984525],[4,775.8,1436.3,0,0,6737.538,-289.1815],[5,775.8,1441.7,0,0,6661.772,-291.7797],[6,762.1,1421.6,0,0,6738.859,-269.1307],[7,762.1,1425.2,0,0,6666.563,-270.8753],[8,762.1,1429,0,0,6663.543,-273.1828],[9,762.1,1432.9,0,0,6660.444,-275.5511],[10,762.1,1436.3,0,0,6638.274,-278.5977],[11,762.1,1441.7,0,0,6633.765,-281.5688]]}]";
	json j = json::parse(s);
};

TEST_F(TrajectoryTest, correctlySetInitialValues) {
	EXPECT_EQ(tr.sigma, 0.05);
	EXPECT_EQ(tr.timestep, 0.02);
	EXPECT_EQ(tr.planDuration, 1.0);
	EXPECT_EQ(tr.numberOfSamples, 10);
}

TEST_F(TrajectoryTest, DISABLED_correctlyCreatesAlternateGoals) {
	string next_mode = plan.select_mode(v1_, road);
	StateGoal goal = plan.realizePlan(next_mode, v1_, road);
	cout << "next mode : " << next_mode << endl;
	cout << "original goal: " << endl;
	goal.printGoal();
	cout << "pertubed goals: " << endl;
	auto possible_goals = tr.perturbGoal(goal, road);
	for (auto possible_goal : possible_goals) {
		possible_goal.printGoal();
	}
	// next end state {126.5 , 5, 10}
	// next end state {136.5 , 15, 10}
}

TEST_F(TrajectoryTest, correctlyCreateJMTPoly) {
	string next_mode = plan.select_mode(v1_, road);
	StateGoal goal = plan.realizePlan(next_mode, v1_, road);
	auto coeffs = tr.jmt(goal, 1.0, 1);
	Polynomial p(coeffs);
	EXPECT_EQ(p.evalAt(0.0, 0), goal.start_s[0]);
	EXPECT_EQ(p.evalAt(0.0, 1), goal.start_s[1]);
	EXPECT_EQ(p.evalAt(0.0, 2), goal.start_s[2]);
	EXPECT_NEAR(p.evalAt(1.0, 0), goal.end_s[0], 0.1);
	EXPECT_NEAR(p.evalAt(1.0, 1), goal.end_s[1], 0.1);
	EXPECT_NEAR(p.evalAt(1.0, 2), goal.end_s[2], 0.1);
	// for d now
	coeffs = tr.jmt(goal, 1.0, 2);
	Polynomial q(coeffs);
	EXPECT_EQ(q.evalAt(0.0, 0), goal.start_d[0]);
	EXPECT_EQ(q.evalAt(0.0, 1), goal.start_d[1]);
	EXPECT_EQ(q.evalAt(0.0, 2), goal.start_d[2]);
	EXPECT_NEAR(q.evalAt(1.0, 0), goal.end_d[0], 0.1);
	EXPECT_NEAR(q.evalAt(1.0, 1), goal.end_d[1], 0.1);
	EXPECT_NEAR(q.evalAt(1.0, 2), goal.end_d[2], 0.1);
}

TEST_F(TrajectoryTest, correctlyGeneratesTrajectory) {
	string next_mode = plan.select_mode(v1_, road);
	StateGoal goal = plan.realizePlan(next_mode, v1_, road);
	auto some_curve = tr.generateTrajectory(goal, v1_, road);
	// some_curve.printTrajectory();
	Polynomial p(some_curve.s_trajectory);
	Polynomial q(some_curve.d_trajectory);
	EXPECT_EQ(p.evalAt(0.0, 0), goal.start_s[0]);
	EXPECT_EQ(p.evalAt(0.0, 1), goal.start_s[1]);
	EXPECT_EQ(p.evalAt(0.0, 2), goal.start_s[2]);
	EXPECT_EQ(q.evalAt(0.0, 0), goal.start_d[0]);
	EXPECT_EQ(q.evalAt(0.0, 1), goal.start_d[1]);
	EXPECT_EQ(q.evalAt(0.0, 2), goal.start_d[2]);
	// end states
	EXPECT_NEAR(p.evalAt(1.0, 0), goal.end_s[0], max(goal.end_s[0] * tr.sigma * 3, tr.sigma * 3));
	EXPECT_NEAR(p.evalAt(1.0, 1), goal.end_s[1], max(goal.end_s[1] * tr.sigma * 3, tr.sigma * 3));
	EXPECT_NEAR(p.evalAt(1.0, 2), goal.end_s[2], max(goal.end_s[2] * tr.sigma * 3, tr.sigma * 3));
	EXPECT_NEAR(q.evalAt(1.0, 0), goal.end_d[0], max(goal.end_d[0] * tr.sigma * 3, tr.sigma * 3));
	EXPECT_NEAR(q.evalAt(1.0, 1), goal.end_d[1], max(goal.end_d[1] * tr.sigma * 3, tr.sigma * 3));
	// EXPECT_NEAR(q.evalAt(1.0, 2), goal.end_d[2], max(goal.end_d[2] * tr.sigma * 3, tr.sigma * 3));
}

class DiscreteCurvesTest : public ::testing::Test {
protected:
	virtual void SetUp() {
		RoadConfiguration rcfg;
		road.updateData(j, 1);
		road.readWayPointsFromFile(map_file_);
		v1_.updateData(j, 1);
		v1_.readPreviousPath(j, 1);
		v1_.useRoadConfiguration(rcfg);
	}

	// virtual void TearDown() {}
	Vehicle v1_;
	Vehicle v2_;
	Road  road;
	Planner plan;
	TrajectoryGenerator tr;
	CurveHandler handler;

	string map_file_ = "C:/Users/buffo/Code/python/prj - selfDrivingCars/carnd-project/11.path.planning/data/highway_map.csv"; // Waypoint map to read from
	string s = "[\"telemetry\",{\"x\":909.48,\"y\":1128.67,\"yaw\":0,\"speed\":0,\"s\":124.8336,\"d\":6.164833,\"previous_path_x\":[],\"previous_path_y\":[],\"end_path_s\":0,\"end_path_d\":0,\"sensor_fusion\":[[0,826.3896,1128.832,23.23647,-0.1320468,41.80027,6.090794],[1,854.8988,1128.887,20.84263,-0.05619142,70.32576,5.989493],[2,1036.174,1155.92,17.97254,7.392407,253.5273,5.962598],[3,1031.829,1149.801,18.13678,7.502246,247.1916,9.984525],[4,775.8,1436.3,0,0,6737.538,-289.1815],[5,775.8,1441.7,0,0,6661.772,-291.7797],[6,762.1,1421.6,0,0,6738.859,-269.1307],[7,762.1,1425.2,0,0,6666.563,-270.8753],[8,762.1,1429,0,0,6663.543,-273.1828],[9,762.1,1432.9,0,0,6660.444,-275.5511],[10,762.1,1436.3,0,0,6638.274,-278.5977],[11,762.1,1441.7,0,0,6633.765,-281.5688]]}]";
	json j = json::parse(s);
};

TEST_F(DiscreteCurvesTest, correctlyCreatesCurvesInFrenet) {
	string next_mode = plan.select_mode(v1_, road);
	StateGoal goal = plan.realizePlan(next_mode, v1_, road);
	auto some_curve = tr.generateTrajectory(goal, v1_, road);
	auto curve = handler.createCurveFromCoefficientsInFrenet(some_curve, 50);
	// curve.printCurve();
	SUCCEED();
}

TEST_F(DiscreteCurvesTest, correctlyCreatesCurvesInXY) {
	string next_mode = plan.select_mode(v1_, road);
	StateGoal goal = plan.realizePlan(next_mode, v1_, road);
	auto some_curve = tr.generateTrajectory(goal, v1_, road);
	auto curve = handler.createCurveFromCoefficientsInXY(some_curve, 50, road.wpts);
	// curve.printCurve();
	SUCCEED();
}

TEST_F(DiscreteCurvesTest, correctlyCreatesCurvestoJSON) {
	string next_mode = plan.select_mode(v1_, road);
	StateGoal goal = plan.realizePlan(next_mode, v1_, road);
	auto some_curve = tr.generateTrajectory(goal, v1_, road);
	auto curve = handler.createCurveFromCoefficientsInXY(some_curve, 50, road.wpts);
	curve = handler.mergeCurves(curve, v1_.previousCurve);
	// cout << curve.toJson() << endl;
	SUCCEED();
}



TEST_F(DiscreteCurvesTest, FrenetToXYCoordinate) {
	auto xy = coords::getXY(126.5, 6.16483, road.wpts);
	cout << "X : " << xy[0] << "Y : " << xy[1] << endl;

	auto sd = coords::getFrenet(911.145, 1128.85, coords::deg2rad(v1_.yaw), road.wpts);
	cout << "S : " << sd[0] << "D : " << sd[1] << endl;
}