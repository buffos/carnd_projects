#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "vehicle.h"
#include "road.h"

using namespace std;

using ::testing::ElementsAre;
using ::testing::DoubleNear;


class VehicleTest : public ::testing::Test {
protected:
	virtual void SetUp() {
		RoadConfiguration rcfg;
		v1_.updateData(j, 1, true);
		v1_.readPreviousPath(j, 1);
		v1_.useRoadConfiguration(rcfg);
		road.updateData(j, 1);
		v2_ = road.cars[0];
	}

	// virtual void TearDown() {}
	Vehicle v0_;
	Vehicle v1_;
	Vehicle v2_;
	Road  road;

	string s = "[\"telemetry\",{\"x\":909.48,\"y\":1128.67,\"yaw\":15,\"speed\":0,\"s\":124.8336,\"d\":6.164833,\"previous_path_x\":[],\"previous_path_y\":[],\"end_path_s\":0,\"end_path_d\":0,\"sensor_fusion\":[[0,826.3896,1128.832,23.23647,-0.1320468,41.80027,6.090794],[1,854.8988,1128.887,20.84263,-0.05619142,70.32576,5.989493],[2,1036.174,1155.92,17.97254,7.392407,253.5273,5.962598],[3,1031.829,1149.801,18.13678,7.502246,247.1916,9.984525],[4,775.8,1436.3,0,0,6737.538,-289.1815],[5,775.8,1441.7,0,0,6661.772,-291.7797],[6,762.1,1421.6,0,0,6738.859,-269.1307],[7,762.1,1425.2,0,0,6666.563,-270.8753],[8,762.1,1429,0,0,6663.543,-273.1828],[9,762.1,1432.9,0,0,6660.444,-275.5511],[10,762.1,1436.3,0,0,6638.274,-278.5977],[11,762.1,1441.7,0,0,6633.765,-281.5688]]}]";
	json j = json::parse(s);
};

TEST_F(VehicleTest, hasInitialValues) {
	EXPECT_EQ(v0_.carLength , 5.0);
	EXPECT_EQ(v0_.x, 0.0);
	EXPECT_EQ(v0_.y, 0.0);
	EXPECT_EQ(v0_.s, 0.0);
	EXPECT_EQ(v0_.d, 0.0);
	EXPECT_EQ(v0_.yaw, 0.0);
	EXPECT_EQ(v0_.speed, 0.0);
	EXPECT_EQ(v0_.acc, 0.0);
	EXPECT_EQ(v0_.end_s, 0.0);
	EXPECT_EQ(v0_.end_d, 0.0);
	EXPECT_EQ(v0_.init_clock, false);
	EXPECT_STREQ(v0_.mode.c_str(), "KL");
	EXPECT_EQ(v0_.previousCurve.c_1.size(), 0);
	EXPECT_EQ(v0_.r.lanes, 3); // rcfg is the default one
	EXPECT_THAT(v0_.currentGoal.start_s, ElementsAre(0, 0, 0)); 
	EXPECT_THAT(v0_.currentGoal.start_d, ElementsAre(0, 0, 0));
	EXPECT_THAT(v0_.currentGoal.start_s, ElementsAre(0, 0, 0));
	EXPECT_THAT(v0_.currentGoal.start_s, ElementsAre(0, 0, 0));
}


TEST_F(VehicleTest, initializesFromJson) {
	EXPECT_EQ(v1_.carLength, 5.0);
	EXPECT_EQ(v1_.x, 909.48);
	EXPECT_EQ(v1_.y, 1128.67);
	EXPECT_EQ(v1_.s, 124.8336);
	EXPECT_EQ(v1_.d, 6.164833);
	EXPECT_NEAR(v1_.yaw, 0.261799, 0.001);
	EXPECT_EQ(v1_.speed, 0.0);
	EXPECT_EQ(v1_.acc, 0.0);
	EXPECT_EQ(v1_.end_s, 0.0);
	EXPECT_EQ(v1_.end_d, 0.0);
	EXPECT_EQ(v1_.init_clock, true);
	EXPECT_STREQ(v1_.mode.c_str(), "KL");
	EXPECT_EQ(v1_.previousCurve.c_1.size(), 0);
	EXPECT_NE(chrono::duration_cast<std::chrono::microseconds>(v1_.time - chrono::steady_clock::now()).count(), 0);
	EXPECT_THAT(v0_.currentGoal.start_s, ElementsAre(0, 0, 0));
	EXPECT_THAT(v0_.currentGoal.start_d, ElementsAre(0, 0, 0));
	EXPECT_THAT(v0_.currentGoal.start_s, ElementsAre(0, 0, 0));
	EXPECT_THAT(v0_.currentGoal.start_s, ElementsAre(0, 0, 0));
}

TEST_F(VehicleTest, returnsTheCorrectLaneNumber) {
	EXPECT_EQ(v1_.getLane(), 2);
	v1_.d = 3.0;
	EXPECT_EQ(v1_.getLane(), 1);
	v1_.d = 7.0;
	EXPECT_EQ(v1_.getLane(), 2);
	v1_.d = 9.0;
	EXPECT_EQ(v1_.getLane(), 3);
	v1_.d = 8.0;
	EXPECT_EQ(v1_.getLane(), 3);
	v1_.d = 12.0;
	EXPECT_EQ(v1_.getLane(), 0);  // out of the road
}

TEST_F(VehicleTest, returnsTheCorrectStateAtTime) {
	auto state = v1_.getStateAt(0);

	EXPECT_THAT(state, ElementsAre(2, 124.8336, 0.0, 0.0)); // lane , s, v, acc

	v1_.speed = 10; // m/s
	v1_.acc = 0.5;
	state = v1_.getStateAt(1); // state after 1 sec

	EXPECT_THAT(state, ElementsAre(2, DoubleNear(124.8336 + 10 * 1 + 0.5 * 1 / 2, 0.1), 10.5, 0.5)); // lane , s, v, acc

	state = v1_.getStateAt(2); // state after 2 sec
	EXPECT_THAT(state, ElementsAre(2, DoubleNear(124.8336 + 10 * 2 + 0.5 * 2 * 2 / 2, 0.1), 11.0, 0.5)); // lane , s, v, acc
}

TEST_F(VehicleTest, checksCollisionAtTime) {
	v1_.s = 100;
	v2_.s = 0;
	v1_.speed = 0;
	v2_.speed = 10;
	v1_.acc = 0;
	v2_.acc = 0;

	EXPECT_FALSE(v1_.collidesWith(v2_, 1));
	EXPECT_FALSE(v1_.collidesWith(v2_, 2));
	EXPECT_TRUE(v1_.collidesWith(v2_, 10));

	v1_.speed = 5; // both cars rolling
	EXPECT_TRUE(v1_.collidesWith(v2_, 20));
	v1_.d = 3.0; // change lane
	EXPECT_FALSE(v1_.collidesWith(v2_, 20));
}

TEST_F(VehicleTest, willCollideWithAtTime){
	v1_.s = 100;
	v2_.s = 0;
	v1_.speed = 0;
	v2_.speed = 10;
	v1_.acc = 0;
	v2_.acc = 0;

	auto result = v1_.willCollideWith(v2_, 20, 1); // find if there is collision in 20*1 sec
	EXPECT_TRUE(result.first);
	EXPECT_EQ(result.second, 10);

	result = v1_.willCollideWith(v2_, 20, 0.1); // find if there is collision in 20*0.1 = 2 sec
	EXPECT_FALSE(result.first);
	EXPECT_EQ(result.second, -1);

	v1_.d = 3.0; // change lane
	result = v1_.willCollideWith(v2_, 20, 1); // find if there is collision in 20*0.1 = 2 sec
	EXPECT_FALSE(result.first);
	EXPECT_EQ(result.second, -1);
}