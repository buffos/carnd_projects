#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "road.h"

using namespace std;



class RoadTest : public ::testing::Test {
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

	string map_file_ = "C:/Users/buffo/Code/python/prj - selfDrivingCars/carnd-project/11.path.planning/data/highway_map.csv"; // Waypoint map to read from
	string s = "[\"telemetry\",{\"x\":909.48,\"y\":1128.67,\"yaw\":0,\"speed\":0,\"s\":124.8336,\"d\":6.164833,\"previous_path_x\":[],\"previous_path_y\":[],\"end_path_s\":0,\"end_path_d\":0,\"sensor_fusion\":[[0,826.3896,1128.832,23.23647,-0.1320468,41.80027,6.090794],[1,854.8988,1128.887,20.84263,-0.05619142,70.32576,5.989493],[2,1036.174,1155.92,17.97254,7.392407,253.5273,5.962598],[3,1031.829,1149.801,18.13678,7.502246,247.1916,9.984525],[4,775.8,1436.3,0,0,6737.538,-289.1815],[5,775.8,1441.7,0,0,6661.772,-291.7797],[6,762.1,1421.6,0,0,6738.859,-269.1307],[7,762.1,1425.2,0,0,6666.563,-270.8753],[8,762.1,1429,0,0,6663.543,-273.1828],[9,762.1,1432.9,0,0,6660.444,-275.5511],[10,762.1,1436.3,0,0,6638.274,-278.5977],[11,762.1,1441.7,0,0,6633.765,-281.5688]]}]";
	json j = json::parse(s);
};

TEST_F(RoadTest, correctlyReadingWaypoints) {
	int n = road.wpts.size();
	EXPECT_EQ(n, 181);
	// first point checking
	EXPECT_NEAR(road.wpts[0].dx, -0.02359831, 0.001);
	EXPECT_NEAR(road.wpts[0].dy, -0.9997216, 0.001);
	EXPECT_NEAR(road.wpts[0].x, 784.6001, 0.001);
	EXPECT_NEAR(road.wpts[0].y, 1135.571, 0.001);
	EXPECT_NEAR(road.wpts[0].s, 0, 0.001);
	// last point checking
	EXPECT_NEAR(road.wpts[n - 1].dx, -0.107399, 0.001);
	EXPECT_NEAR(road.wpts[n - 1].dy, -0.9942161, 0.001);
	EXPECT_NEAR(road.wpts[n - 1].x, 753.2067, 0.001);
	EXPECT_NEAR(road.wpts[n - 1].y, 1136.417, 0.001);
	EXPECT_NEAR(road.wpts[n - 1].s, 6914.14925765991, 0.001);
}

TEST_F(RoadTest, correctlyImportingSensorFusion) {
	EXPECT_EQ(road.cars.size(), 12);
	EXPECT_NEAR(road.cars[1].x, 854.8988, 0.001);
	EXPECT_NEAR(road.cars[1].y, 1128.887, 0.001);
	EXPECT_NEAR(road.cars[1].speed, sqrt(20.84263 * 20.84263 + 0.05619142 * 0.05619142), 0.001); //[1,854.8988,1128.887,20.84263,-0.05619142,70.32576,5.989493]
	EXPECT_NEAR(road.cars[1].yaw, -0.05619142 / 20.84263, 0.001);
}

TEST_F(RoadTest, correctlyFindClosestVehicleInFront) {
	auto d1 = road.distanceInFront(v1_, 1);
	auto d2 = road.distanceInFront(v1_, 2);
	auto d3 = road.distanceInFront(v1_, 3);

	//cout << "lane: " << road.cars[0].getLane() << " speed: " << road.cars[0].speed << endl;
	//cout << "lane: " << road.cars[1].getLane() << " speed: " << road.cars[1].speed << endl;
	//cout << "lane: " << road.cars[2].getLane() << " speed: " << road.cars[2].speed << endl;
	//cout << "lane: " << road.cars[3].getLane() << " speed: " << road.cars[3].speed << endl;
	//cout << "lane: " << road.cars[4].getLane() << " speed: " << road.cars[4].speed << endl;
	//cout << "lane: " << road.cars[5].getLane() << " speed: " << road.cars[5].speed << endl;

	// for lane 1 , no car in lane 1
	EXPECT_NEAR(d1[0], 9999999.0, 0.001); // distance
	EXPECT_NEAR(d1[1], 1000000.0, 0.001); // speed

	// for lane 2 , carId == 2
	EXPECT_NEAR(d2[0], 253.52 - 124.83336 - v1_.carLength, 0.1); // distance with carId: 2
	EXPECT_NEAR(d2[1], 19.4335, 0.001); // speed

	// for lane 3 , carId == 3
	EXPECT_NEAR(d3[0], 247.19 - 124.83336 - v1_.carLength, 0.1); // distance with carId: 3
	EXPECT_NEAR(d3[1], 19.6272, 0.001); // speed
}

TEST_F(RoadTest, correctlyBehindClosestVehicleInFront) {
	auto d1 = road.distanceBehind(v1_, 1);
	auto d2 = road.distanceBehind(v1_, 2);
	auto d3 = road.distanceBehind(v1_, 3);


	// for lane 1 , no car in lane 1
	EXPECT_NEAR(d1[0], 9999999.0, 0.001); // distance
	EXPECT_NEAR(d1[1], 1000000.0, 0.001); // speed

	// for lane 2 , carId == 1
	EXPECT_NEAR(d2[0], 124, 83336 - 70.32 - v1_.carLength, 0.1); // distance with carId: 2
	EXPECT_NEAR(d2[1], 20.8427, 0.1); // speed

	// for lane 3 , no car in lane 3 behind. just a car in front
	EXPECT_NEAR(d3[0], 9999999.0, 0.001); // distance
	EXPECT_NEAR(d3[1], 1000000.0, 0.001); // speed
}

TEST_F(RoadTest, correctlyFindClosestVehicleAtTimeTandPositionS) {
	//auto s_distance = coords::real_s_distance(road.cars[1].getStateAt(0)[1], 124.8336, road.rcfg.max_s)[0];
	//auto d_distance = road.cars[2].d - 6.164;
	//
	//cout << "car with id  = 1 " << road.cars[1].getStateAt(0)[1] << " d: " << road.cars[1].d << endl;	
	//cout << "car speed: " << road.cars[1].getStateAt(1)[2] << " " << road.cars[1].speed;
	//cout << "real s distance: " << s_distance << endl;
	//cout << "real d distance: " << d_distance << endl;
	//cout << sqrt(s_distance * s_distance + d_distance * d_distance) << endl;
	//cout << road.closestVehicleAt(124.8336, 6.164, 0) << endl;

	EXPECT_NEAR(road.closestVehicleAt(251.0, 6.0, 0), 2.52, 0.1);
	EXPECT_NEAR(road.closestVehicleAt(124.8336, 6.164, 0), 54.508, 0.1); // closest to my car at t = 0
	EXPECT_NEAR(road.closestVehicleAt(124.8336, 6.164, 1), 54.508 - 20.8427 * 1, 0.1); // closest to my car at t = 1 (20.8427 is the speed of the car
	EXPECT_NEAR(road.closestVehicleAt(124.8336, 6.164, 2), 54.508 - 20.8427 * 2, 0.1); // closest to my car at t = 2
}
