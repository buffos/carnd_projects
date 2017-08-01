#ifndef CONSTANTS_H_
#define CONSTANTS_H_

namespace constants
{
// road constants
const double MAX_SPEED = 22.00;        // 50mph = 22.352 m /s
const double TARGET_SPEED = 20.00;     // 45mph
const double MAX_ACCELERATION = 10.00; // 10m/s^2
const double MAX_JERK = 30.00;         // 50/m/s^3
const int FRAMES_PER_SEC = 50;         // 0.02 seconds per frame
const int DEFAULT_LANES = 3;           // 3 lanes default value
const double DEFAULT_LANE_WIDTH = 4.0; // 4m default width lane

// planner constants
const double MAX_COST = 100000.0;    // a big number
const double PLAN_DURATION = 5.0;    // plan duration in seconds
const double SAFETY_DISTANCE = 10.0; // how close to other vehicles to go in meters

const double WEIGHT_NEED_FOR_SPEED = 100.; // *TimeToReachFrontVehicle
const double WEIGHT_NEED_FOR_SPACE = 1000.; // *SpaceAround

// trajectories constant
const double STD_DEVIATION = 0.05; // how different should be the values from the mean
const int SAMPLE_TRAJECTORIES = 10; // how many trajectories to generate in order to pick the best

const double WEIGHT_TIME_DIFFERENCE_COST = 1.0;
const double WEIGHT_S_DIFFERENCE_COST = 100.0;
const double WEIGHT_D_DIFFERENCE_COST = 10000.0;
const double WEIGHT_COLLISION_COST = 1000000.0;
const double WEIGHT_BUFFER_COST = 100.0;
const double WEIGHT_MAX_ACCELERATION_COST = 5.0;
const double WEIGHT_MAX_JERK_COST = 5.0;

// splines constants
const double OUT_OF_BOUNDS = 100000000.0;
}

#endif // CONSTANTS_H_