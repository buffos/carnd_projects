#ifndef CONSTANTS_H_
#define CONSTANTS_H_

namespace constants {
const double CAR_WIDTH = 2.0; // in meters
const double CAR_LENGTH = 5.0; // in meters
// road constants
const double TRACKLENGTH = 6945.554;
const double MAX_SPEED = 20.00;        // 50mph = 22.352 m /s
const double TARGET_SPEED = 19.50;     // 45mph
const double MIN_SPEED = 10.00;         // 22mph
const double MAX_ACCELERATION = 10.00; // 10m/s^2
const double MAX_JERK = 10.00;         // 10/m/s^3
const int FRAMES_PER_SEC = 50;         // 0.02 seconds per frame
const int DEFAULT_LANES = 3;           // 3 lanes default value
const double DEFAULT_LANE_WIDTH = 4.0; // 4m default width lane
const double MAX_DISTANCE_TO_TRACK = 100; // distance between cars

// planner constants
const double MAX_COST = 100000.0;              // a big number
const double PLAN_DURATION = 2.0;              // plan duration in seconds
const double MF_DURATION = 2.0;                // MatchFront plan duration in seconds
const double CL_DURATION = 2.0;                // Change Lanes plan duration in seconds
const double START_DURATION = 4.0;             // Start Engine plan duration in seconds
const double SAFETY_DISTANCE = 2 * CAR_LENGTH; // how close to other vehicles to go in meters
const double SAFETY_WIDTH = CAR_WIDTH / 2;
const int UPDATE_WHEN = 40;                // update when 40 points are left in curve

const double WEIGHT_NEED_FOR_SPEED = 100.; // *TimeToReachFrontVehicle
const double WEIGHT_NEED_FOR_SPACE = 1000.; // *SpaceAround

// trajectories constant
const double STD_DEVIATION = 0.10; // how different should be the values from the mean
const int SAMPLE_TRAJECTORIES = 20; // how many trajectories to generate in order to pick the best

const double WEIGHT_TIME_DIFFERENCE_COST = 1.0;
const double WEIGHT_S_DIFFERENCE_COST = 100.0;
const double WEIGHT_D_DIFFERENCE_COST = 100.0;
const double WEIGHT_COLLISION_COST = 10000000.0;
const double WEIGHT_BUFFER_COST = 140.0;
const double WEIGHT_MAX_ACCELERATION_COST = 10000000.0;
const double WEIGHT_MAX_JERK_COST = 10000000.0;
const double WEIGHT_MAX_SPEED_COST = 10000000.0;
const double WEIGHT_EFFICIENCY_COST = 110.0;
const double WEIGHT_TOTAL_ACCELERATION_COST = 100.0;
const double WEIGHT_TOTAL_JERK_COST = 100.0;

// splines constants
const double OUT_OF_BOUNDS = 100000000.0;
};

#endif // CONSTANTS_H_