#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "../include/vehicle.h"
#include "../include/road.h"
#include "../include/planner.h"


using namespace std;

using namespace testing;


class PlannerTest : public ::testing::Test {
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

	string map_file_ = "C:/Users/buffo/Code/python/prj - selfDrivingCars/carnd-project/11.path.planning/data/highway_map.csv"; // Waypoint map to read from
	string s = "[\"telemetry\",{\"x\":909.48,\"y\":1128.67,\"yaw\":0,\"speed\":0,\"s\":124.8336,\"d\":6.164833,\"previous_path_x\":[],\"previous_path_y\":[],\"end_path_s\":0,\"end_path_d\":0,\"sensor_fusion\":[[0,826.3896,1128.832,23.23647,-0.1320468,41.80027,6.090794],[1,854.8988,1128.887,20.84263,-0.05619142,70.32576,5.989493],[2,1036.174,1155.92,17.97254,7.392407,253.5273,5.962598],[3,1031.829,1149.801,18.13678,7.502246,247.1916,9.984525],[4,775.8,1436.3,0,0,6737.538,-289.1815],[5,775.8,1441.7,0,0,6661.772,-291.7797],[6,762.1,1421.6,0,0,6738.859,-269.1307],[7,762.1,1425.2,0,0,6666.563,-270.8753],[8,762.1,1429,0,0,6663.543,-273.1828],[9,762.1,1432.9,0,0,6660.444,-275.5511],[10,762.1,1436.3,0,0,6638.274,-278.5977],[11,762.1,1441.7,0,0,6633.765,-281.5688]]}]";
	json j = json::parse(s);
};

TEST_F(PlannerTest, getsInitialValues) {
	EXPECT_EQ(constants::SAFETY_DISTANCE, constants::SAFETY_DISTANCE);

	EXPECT_THAT(plan.next_modes, Contains(Key("KL")));
	EXPECT_THAT(plan.next_modes, Contains(Key("MF")));
	EXPECT_THAT(plan.next_modes, Contains(Key("LCL")));
	EXPECT_THAT(plan.next_modes, Contains(Key("LCR")));

	EXPECT_THAT(plan.next_modes["KL"], ElementsAreArray({ "KL", "LCL", "LCR", "MF" }));
	EXPECT_THAT(plan.next_modes["MF"], ElementsAreArray({ "MF", "KL", "LCL", "LCR" }));
	EXPECT_THAT(plan.next_modes["LCL"], ElementsAreArray({ "KL" }));
	EXPECT_THAT(plan.next_modes["LCR"], ElementsAreArray({ "KL" }));
	EXPECT_THAT(plan.next_modes["XYZ"], IsEmpty());
}
/*

TEST_F(PlannerTest, correctlyCalculatesCostSpeed) {
	// costSpeed ofr a car that goes with 25m/s has 100m free road ahead and front car drives with 25m/s
	EXPECT_EQ(plan.costSpeed(v1_, 25.0, 100.0, 25.0), 0.0); // same speed with front car. no cost
	EXPECT_EQ(plan.costSpeed(v1_, 15.0, 100.0, 25.0), 0.0); // lower speed with front car. no cost
	double timeToReachFrontCar = (100 - constants::SAFETY_DISTANCE) / 25.0;
	EXPECT_NEAR(plan.costSpeed(v1_, 25.0, 100.0, 0.0), 100 * timeToReachFrontCar, 0.1); // car in front is slow or stopped.
}

TEST_F(PlannerTest, correctlyCalculatesCostMatchFrontSpeed) {
	// front car is car with id:2
	EXPECT_EQ(plan.costMatchFrontSpeed(v1_, road), 0.0); // my car is stopped and the front car is faster.
	v1_.speed = 19.4335 - 1; // match almost front car speed
	EXPECT_EQ(plan.costMatchFrontSpeed(v1_, road), 0.0); // my car is stopped and the front car is faster.
	v1_.speed = 19.4335; // match almost front car speed
	EXPECT_NEAR(plan.costMatchFrontSpeed(v1_, road), 100 * (road.rcfg.target_speed - 19.4335), 0.1); // my car is stopped and the front car is faster.

}

TEST_F(PlannerTest, correctlyCalculatesCostKeepLane) {
	// front car is car with id:2
	auto d2 = road.distanceInFront(v1_, v1_.getLane());
	EXPECT_NEAR(d2[0], 123.69, 0.1); // distance with carId: 2
	EXPECT_NEAR(d2[1], 19.4335, 0.001); // speed
	double costForSpeed = plan.costSpeed(v1_, road.rcfg.target_speed, d2[0], d2[1]); // already tested method
	double totalCost = costForSpace + costForSpeed;
	EXPECT_NEAR(plan.costKeepLane(v1_, road), totalCost, 0.01);
}

TEST_F(PlannerTest, correctlyCalculatesCostChangeLaneLeft) {
	// there is no front car in lane 1
	EXPECT_NEAR(plan.costLaneChangeLeft(v1_, road), 0.0, 0.01);
}

TEST_F(PlannerTest, correctlyCalculatesCostChangeLaneRight) {
	// in front I have car with id: 3 and no car behind
	auto d3 = road.distanceInFront(v1_, 3); // I am in lane 2 so I go to lane 3
	EXPECT_NEAR(d3[0], 117.36, 0.1); // distance with carId: 3
	EXPECT_NEAR(d3[1], 19.6272, 0.001); // speed
	double costForSpace = plan.costSpace(v1_, constants::SAFETY_DISTANCE, d3[0], 10000000.0); // I have plenty of space in front and behind
	double costForSpeed = plan.costInLane(v1_, road.rcfg.target_speed, d3[0], d3[1]); // The cost to go to target speed in that lane.
	EXPECT_NEAR(plan.costLaneChangeRight(v1_, road), costForSpace + costForSpeed, 0.01);
	// now lets speed up and go fast
	v1_.speed = constants::TARGET_SPEED; // at target speed
	double timeToReachFrontVehicle = (d3[0] - constants::SAFETY_DISTANCE) / (road.rcfg.target_speed - d3[1]); // distance / how faster I am going
	costForSpeed = 100 * timeToReachFrontVehicle;
	EXPECT_NEAR(plan.costLaneChangeRight(v1_, road), costForSpace + costForSpeed, 0.01);
	// now lets go close to the vehicle
	v1_.s = road.cars[3].s - v1_.carLength - 2; // just 2 meters away;
	d3 = road.distanceInFront(v1_, 3); // recalulate distance inf ront
	timeToReachFrontVehicle = (d3[0] - constants::SAFETY_DISTANCE) / (road.rcfg.target_speed - d3[1]); // and recalculate cost for speed.
	timeToReachFrontVehicle = (timeToReachFrontVehicle < 0) ? 0.0 : timeToReachFrontVehicle;
	costForSpeed = constants::WEIGHT_NEED_FOR_SPEED * timeToReachFrontVehicle;
	costForSpace = plan.costSpace(v1_, constants::SAFETY_DISTANCE, d3[0], 10000000.0);
	EXPECT_NEAR(plan.costLaneChangeRight(v1_, road), costForSpace + costForSpeed, 0.01);
}

TEST_F(PlannerTest, correctlyCalculateCostForSpace) {
	auto cost = plan.costSpace(v1_, 2.0, 10000000.0); // 2 meters in front space and I need 4 for safety
	EXPECT_NEAR(cost, 2000.0, 0.01);
	cost = plan.costSpace(v1_, 2.0, 0.0); // do not cause error with one value being zero
	EXPECT_NEAR(cost, 2000.0, 0.01);
	cost = plan.costSpace(v1_, 0.0, 2.0); // do not cause error with one value being zero
	EXPECT_NEAR(cost, 2000.0, 0.01);
	cost = plan.costSpace(v1_, 0.0, 0.0); // max cost when both are zero
	EXPECT_NEAR(cost, constants::SAFETY_DISTANCE, 0.01); // collision

}

TEST_F(PlannerTest, correctlySelectsMode) {
	//cout << "Cost change left: " << plan.costLaneChangeLeft(v1_, road) << endl;
	//cout << "Cost change right: " << plan.costLaneChangeRight(v1_, road) << endl;
	//cout << "Cost Keep Lane: " << plan.costKeepLane(v1_, road) << endl;
	//cout << "Cost Match Front Speed: " << plan.costMatchFrontSpeed(v1_, road) << endl;
	EXPECT_EQ(plan.select_mode(v1_, road), "MF"); // I am going slow (0 speed) , so car in front is not a problem
	v1_.d = 9; // changing to lane 3;
	v1_.speed = 25; // going faster than the front car
	//cout << "Cost change left: " << plan.costLaneChangeLeft(v1_, road) << endl; 
	//cout << "Cost change right: " << plan.costLaneChangeRight(v1_, road) << endl;
	//cout << "Cost Keep Lane: " << plan.costKeepLane(v1_, road) << endl;
	//cout << "Cost Match Front Speed: " << plan.costMatchFrontSpeed(v1_, road) << endl;
	//cout << plan.select_mode(v1_, road);
	EXPECT_EQ(plan.select_mode(v1_, road), "MF"); // slow down to match front car speed and wait till I can go left.
	v1_.d = 6; // back to lane 2 but now I am going fast and lane 1 is empty
	EXPECT_EQ(plan.select_mode(v1_, road), "LCL"); // select lane 1 to switch.
}

TEST_F(PlannerTest, correctlyRealizePlanKeepLane) {
	v1_.speed = 10.0; // lets give some speed.
	v1_.acc = 10;
	auto newGoal = plan.realizeKeepLane(v1_, road);
	EXPECT_THAT(newGoal.start_s, ElementsAreArray({ v1_.s , v1_.speed, v1_.acc }));
	EXPECT_THAT(newGoal.start_d, ElementsAreArray({ v1_.d , 0.0, 0.0 })); // maybe d_speed should match local_yaw
	EXPECT_THAT(newGoal.end_s, ElementsAreArray({ DoubleNear(139.834, 0.1), DoubleNear(20.0, 0.1), DoubleNear(10.0, 0.1) }));
	EXPECT_THAT(newGoal.end_d, ElementsAreArray({ 6.0, 0.0, 0.0 })); // I want to move to the center of the current lane as much as possible
}

TEST_F(PlannerTest, correctlyRealizePlanToMatchFrontVehicleSpeed) {
	v1_.speed = 10.0; // lets give some speed.
	v1_.acc = 10.0;
	auto newGoal = plan.realizeMatchFront(v1_, road);
	EXPECT_THAT(newGoal.start_s, ElementsAreArray({ v1_.s , v1_.speed, v1_.acc }));
	EXPECT_THAT(newGoal.start_d, ElementsAreArray({ v1_.d , 0.0, 0.0 })); // maybe d_speed should match local_yaw
	EXPECT_THAT(newGoal.end_s, ElementsAreArray({ DoubleNear(139.55, 0.1), DoubleNear(19.435, 0.1), DoubleNear(9.434, 0.1) }));
	EXPECT_THAT(newGoal.end_d, ElementsAreArray({ 6.0, 0.0, 0.0 })); // I want to move to the center of the current lane as much as possible
}

TEST_F(PlannerTest, correctlyRealizePlanToChangeToLeftLane) {
	v1_.speed = 10.0; // lets give some speed.
	v1_.acc = 10.0;
	auto newGoal = plan.realizeChangeLeft(v1_, road);
	EXPECT_THAT(newGoal.start_s, ElementsAreArray({ v1_.s , v1_.speed, v1_.acc }));
	EXPECT_THAT(newGoal.start_d, ElementsAreArray({ v1_.d , 0.0, 0.0 })); // maybe d_speed should match local_yaw
	// change lanes with constant speed (no acceleration)
	EXPECT_THAT(newGoal.end_s, ElementsAreArray({ DoubleNear(134.834, 0.1), DoubleNear(10.0, 0.1), DoubleNear(0.0, 0.1) }));
	EXPECT_THAT(newGoal.end_d, ElementsAreArray({ v1_.d - road.rcfg.lane_width , 0.0, 0.0 }));
}

TEST_F(PlannerTest, correctlyRealizePlanToChangeToRightLane) {
	v1_.speed = 10.0; // lets give some speed.
	v1_.acc = 10.0;
	auto newGoal = plan.realizeChangeRight(v1_, road);
	EXPECT_THAT(newGoal.start_s, ElementsAreArray({ v1_.s , v1_.speed, v1_.acc }));
	EXPECT_THAT(newGoal.start_d, ElementsAreArray({ v1_.d , 0.0, 0.0 })); // maybe d_speed should match local_yaw
	// change lanes with constant speed (no acceleration)
	EXPECT_THAT(newGoal.end_s, ElementsAreArray({ DoubleNear(134.834, 0.1), DoubleNear(10.0, 0.1), DoubleNear(0.0, 0.1) }));
	EXPECT_THAT(newGoal.end_d, ElementsAreArray({ v1_.d + road.rcfg.lane_width , 0.0, 0.0 }));
}*/
