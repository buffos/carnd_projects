#ifndef CONSTANTS_H_
#define CONSTANTS_H_

namespace constants {
const double CAR_WIDTH = 2.0;          ///< a typical car width in meters
const double CAR_LENGTH = 5.0;         ///<a typical car width in meters
// road constants
const double TRACKLENGTH = 6945.554;   ///< the track length we cover in meters
const double MAX_SPEED = 20.00;        ///< the max allowed speed 50mph = 22.352 m /s
const double TARGET_SPEED = 19.50;     ///< the target speed 45mph
const double MIN_SPEED = 10.00;        ///< the minimum speed. do not go very slowly 22mph
const double MAX_ACCELERATION = 10.00; ///< the maximum allowed instant acceleration 10m/s^2
const double MAX_JERK = 10.00;         ///< the maximum allowed instant jerk 10m/s^3
const int FRAMES_PER_SEC = 50;         ///< 50 frames per second => 1 frame every 0.02sec
const int DEFAULT_LANES = 3;           ///<  3 lanes default value
const double DEFAULT_LANE_WIDTH = 4.0; ///<  4m default width lane

// planner constants
const double MAX_COST = 100000.0;              ///<  a big number
const double PLAN_DURATION = 2.0;              ///<  plan duration in seconds
const double MF_DURATION = 2.0;                ///<  MatchFront plan duration in seconds
const double CL_DURATION = 2.0;                ///<  Change Lanes plan duration in seconds
const double START_DURATION = 4.0;             ///<  Start Engine plan duration in seconds
const double SAFETY_DISTANCE = 2 * CAR_LENGTH; ///<  how close to other vehicles to go in meters
const double SAFETY_WIDTH = CAR_WIDTH / 2;     ///<  the closest that is safe to be with a car by the side
const int UPDATE_WHEN = 40;                    ///<  update when 40 points are left in curve

const double WEIGHT_NEED_FOR_SPEED = 100.;     ///<   multiplier for TimeToReachFrontVehicle
const double WEIGHT_NEED_FOR_SPACE = 1000.;    ///<   multiplier for SpaceAround

// trajectories constant
const double STD_DEVIATION = 0.10;             ///<   how different should the generated values be from the mean
const int SAMPLE_TRAJECTORIES = 20;            ///<   how many trajectories to generate in order to pick the best

const double WEIGHT_TIME_DIFFERENCE_COST = 1.0;        ///<  weight for time difference between original plan and final
const double WEIGHT_S_DIFFERENCE_COST = 100.0;         ///<  weight for s difference between original plan and final
const double WEIGHT_D_DIFFERENCE_COST = 100.0;         ///<  weight for d difference between original plan and final
const double WEIGHT_COLLISION_COST = 10000000.0;       ///<  weight for collision
const double WEIGHT_BUFFER_COST = 140.0;               ///<  weight for avoiding traffic
const double WEIGHT_MAX_ACCELERATION_COST = 10000000.0;///<  weight for not exceeding the allowed acceleration
const double WEIGHT_MAX_JERK_COST = 10000000.0;        ///<  weight for not exceeding the allowed jerk
const double WEIGHT_MAX_SPEED_COST = 10000000.0;       ///<  weight for not exceeding the allowed speed
const double WEIGHT_EFFICIENCY_COST = 110.0;           ///<  weight for going fast on average
const double WEIGHT_TOTAL_ACCELERATION_COST = 100.0;   ///<  weight for not averaging high accelerations over time
const double WEIGHT_TOTAL_JERK_COST = 100.0;           ///<  weight for not averaging high jerk over time

// splines constants
const double OUT_OF_BOUNDS = 100000000.0;              ///<  a value signaling the requested s position is not in spline
};

#endif // CONSTANTS_H_