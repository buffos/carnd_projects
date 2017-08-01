#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "vehicle.h"
#include "road.h"
#include "tools.h"


using namespace std;
using namespace testing;


class SplinesTesting : public ::testing::Test {
protected:
	virtual void SetUp() {
		road.readWayPointsFromFile(map_file_);
	}

	// virtual void TearDown() {}
	Road road;

	string map_file_ = "C:/Users/buffo/Code/python/prj - selfDrivingCars/carnd-project/11.path.planning/data/highway_map.csv"; // Waypoint map to read from

};


TEST_F(SplinesTesting, correctlyCreatesWayPointsIndexInTheMiddle) {

	// middle of waypoints
	auto indexes = coords::getLocalWayPointIndexes(20, 5, 5, road.wpts.size());
	EXPECT_EQ(indexes.size(), 10);
	EXPECT_THAT(indexes, ElementsAreArray({ 16,17,18,19,20,21,22,23,24,25 }));
	// begining
	indexes = coords::getLocalWayPointIndexes(3, 5, 5, road.wpts.size());
	EXPECT_THAT(indexes, ElementsAreArray({ 180,0,1,2,3,4,5,6,7,8 }));
}

TEST_F(SplinesTesting, correctlyCreatesWayPointsIndexInFront) {
	// begining
	auto indexes = coords::getLocalWayPointIndexes(3, 5, 5, road.wpts.size());
	EXPECT_THAT(indexes, ElementsAreArray({ 180,0,1,2,3,4,5,6,7,8 }));
}

TEST_F(SplinesTesting, correctlyCreatesWayPointsIndexInEnd) {
	// begining
	auto indexes = coords::getLocalWayPointIndexes(179, 5, 5, road.wpts.size());
	EXPECT_THAT(indexes, ElementsAreArray({ 175,176,177,178,179,180,0,1,2,3 }));
}

TEST_F(SplinesTesting, correctlyCreatesWayPointsIndexLastWP) {
	// begining
	auto indexes = coords::getLocalWayPointIndexes(180, 5, 5, road.wpts.size());
	EXPECT_THAT(indexes, ElementsAreArray({ 176,177,178,179,180,0,1,2,3,4 }));
}

TEST_F(SplinesTesting, correctlyCreatesWayPointsIndexFirstWP) {
	// begining
	auto indexes = coords::getLocalWayPointIndexes(0, 5, 5, road.wpts.size());
	EXPECT_THAT(indexes, ElementsAreArray({ 177,178,179,180,0,1,2,3,4,5 }));
}

TEST_F(SplinesTesting, correctlyCreatesSplinesAtMiddleWayPoints) {
	double s20 = 565.138957977295; // the s of the 20th way point
	double s25 = 718.46262550354; // the s of the 25th way point
	double s70 = 2212.66669273376; // the s of the 70th way point
	//2234.7 2139.3 2212.66669273376 0.963896 -0.2662791
	unsigned int lastWPindex = road.wpts.size() - 1;
	auto spline = coords::createLocalSplines(s20, road.wpts, road.wpts[lastWPindex].s);
	EXPECT_NEAR(spline.x(s20), 1340.477, 0.1);
	EXPECT_NEAR(spline.y(s20), 1188.307, 0.1);
	EXPECT_NEAR(spline.x(s25), 1492.771, 0.1);
	EXPECT_NEAR(spline.y(s25), 1170.749, 0.1);
	// the correct way to evaluate points
	auto xy = coords::evaluateSplineAtS(s25, 0, spline, road.wpts[lastWPindex].s);
	EXPECT_THAT(xy, ElementsAre(DoubleNear(1492.771, 0.1), DoubleNear(1170.749, 0.1)));
	// OUT OF RANGE should return 
	xy = coords::evaluateSplineAtS(s70, 0, spline, road.wpts[lastWPindex].s);
	EXPECT_THAT(xy, ElementsAre(DoubleNear(1492.771, 0.1), DoubleNear(1170.749, 0.1)));

}