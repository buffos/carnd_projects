#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "../include/vehicle.h"
#include "../include/road.h"
#include "../include/tools.h"


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

TEST_F(SplinesTesting, correctlyFindsIfPointIsInSplice) {
	double s20 = 565.138957977295; // the s of the 20th way point
	double s25 = 718.46262550354; // the s of the 25th way point
	double s70 = 2212.66669273376; // the s of the 70th way point
	double s0 = 0.0; // the s of the 70th way point
	double s3 = 90.4504146575928;
	double s179 = 6871.54959487915;
	double s170 = 6415.47916793823;

	unsigned int lastWPindex = road.wpts.size() - 1;
	double trackLength = road.wpts[lastWPindex].s + 43;

	// normal s_point in middle of the track
	auto spline = coords::createLocalSplines(s20, 5, 5, road.wpts, trackLength);
	EXPECT_TRUE(coords::isPointInSpline(s20, spline));
	EXPECT_FALSE(coords::isPointInSpline(s25, spline)); // just the next one
	EXPECT_FALSE(coords::isPointInSpline(s70, spline));
	EXPECT_FALSE(coords::isPointInSpline(0.0, spline));

	// s_point at the beginning of the track
	spline = coords::createLocalSplines(s0, 5, 5, road.wpts, trackLength);
	EXPECT_TRUE(coords::isPointInSpline(s0, spline));
	EXPECT_TRUE(coords::isPointInSpline(s3, spline));
	EXPECT_TRUE(coords::isPointInSpline(s179, spline));
	EXPECT_FALSE(coords::isPointInSpline(s170, spline));
	EXPECT_FALSE(coords::isPointInSpline(s20, spline));

}

TEST_F(SplinesTesting, correctlyCreatesSplinesAtTheEdges) {
	double s20 = 565.138957977295; // the s of the 20th way point
	double s25 = 718.46262550354; // the s of the 25th way point
	double s70 = 2212.66669273376; // the s of the 70th way point
	double s0 = 0.0; // the s of the 70th way point
	double s3 = 90.4504146575928;
	double s179 = 6871.54959487915;
	double s170 = 6415.47916793823;

	unsigned int lastWPindex = road.wpts.size() - 1;
	double trackLength = road.wpts[lastWPindex].s + 43;

	auto spline = coords::createLocalSplines(s0, 5, 5, road.wpts, trackLength);


	EXPECT_NEAR(spline.x(s0 + trackLength), 784.6001, 0.1);
	EXPECT_NEAR(spline.y(s0 + trackLength), 1135.571, 0.1);
	EXPECT_NEAR(spline.x(s3 + trackLength), 875.0436, 0.1);
	EXPECT_NEAR(spline.y(s3 + trackLength), 1134.808, 0.1);
	EXPECT_NEAR(spline.x(s179), 711.2, 0.1);
	EXPECT_NEAR(spline.y(s179), 1143.5, 0.1);

	// the correct way to evaluate points
	auto xy = coords::evaluateSplineAtS(s0, 0, spline, trackLength);
	EXPECT_THAT(xy, ElementsAre(DoubleNear(784.6001, 0.1), DoubleNear(1135.571, 0.1)));
	xy = coords::evaluateSplineAtS(s3, 0, spline, trackLength);
	EXPECT_THAT(xy, ElementsAre(DoubleNear(875.0436, 0.1), DoubleNear(1134.808, 0.1)));
	xy = coords::evaluateSplineAtS(s179, 0, spline, trackLength);
	EXPECT_THAT(xy, ElementsAre(DoubleNear(711.2, 0.1), DoubleNear(1143.5, 0.1)));
}

TEST_F(SplinesTesting, correctlyCreatesSplinesAtMiddleWayPoints) {
	double s20 = 565.138957977295; // the s of the 20th way point
	double s25 = 718.46262550354; // the s of the 25th way point
	double s70 = 2212.66669273376; // the s of the 70th way point
								   //2234.7 2139.3 2212.66669273376 0.963896 -0.2662791
	unsigned int lastWPindex = road.wpts.size() - 1;
	double trackLength = road.wpts[lastWPindex].s + 43;

	auto spline = coords::createLocalSplines(s20, 5, 10, road.wpts, trackLength);
	EXPECT_NEAR(spline.x(s20), 1340.477, 0.1);
	EXPECT_NEAR(spline.y(s20), 1188.307, 0.1);
	EXPECT_NEAR(spline.x(s25), 1492.771, 0.1);
	EXPECT_NEAR(spline.y(s25), 1170.749, 0.1);
	// the correct way to evaluate points
	auto xy = coords::evaluateSplineAtS(s25, 0, spline, trackLength);
	EXPECT_THAT(xy, ElementsAre(DoubleNear(1492.771, 0.1), DoubleNear(1170.749, 0.1)));
	// OUT OF RANGE should return 
	xy = coords::evaluateSplineAtS(s70, 0, spline, road.wpts[lastWPindex].s);
	EXPECT_THAT(xy, ElementsAre(DoubleNear(constants::OUT_OF_BOUNDS, 0.1), DoubleNear(constants::OUT_OF_BOUNDS, 0.1)));
}