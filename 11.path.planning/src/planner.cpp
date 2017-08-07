/**
 * @file planner.cpp
 * @brief The behavior planner: implementation
 *
 *
 * @author  Kostas Oreopoulos
 */
#include "planner.h"

map<string, vector<string>> Planner::next_modes = {
    {"KL", {"EB", "MF", "KL", "LCL", "LCR"}},
    {"MF", {"EB", "MF", "KL", "LCL", "LCR"}},
    {"EB", {"EB", "MF", "LCL", "LCR"}},
    {"LCL", {"KL"}},
    {"LCR", {"KL"}},
    {"OFF", {"ON"}},
    {"ON", {"KL", "MF", "LCL", "LCR"}}
};

string Planner::select_mode(const Vehicle &car, const Road &r) {
  vector<pair<double, string>> next_modes_cost;
  const vector<string> possible_modes = Planner::next_modes[car.mode]; // a vector holding all the possible next modes
  logger.str(""); //clear log

  for (const string &next_mode : possible_modes) {
    double cost;
    if (next_mode == "LCL") {
      cost = costLaneChange(car, r, true);
    } else if (next_mode == "LCR") {
      cost = costLaneChange(car, r, false);
    } else if (next_mode == "KL") {
      cost = costKeepLane(car, r);
    } else if (next_mode == "MF") {
      cost = costMatchFrontSpeed(car, r);
    } else if (next_mode == "EB") {
      cost = costEmergencyBrake(car, r);
    }
    next_modes_cost.emplace_back(cost, next_mode);
  } // end of examining all possible next modes
  std::sort(next_modes_cost.begin(),
            next_modes_cost.end(),
            [](std::pair<double, string> &i, std::pair<double, string> &j) -> bool {
              return i.first < j.first;
            });
  string selected_mode = next_modes_cost[0].second;
  return selected_mode;
}

double Planner::costLaneChange(const Vehicle &car, const Road &r, bool left) {
  int next = (left) ? -1 : 1;
  double cost = lookBesidesPenalty(car, r, 0); // the cost for this lane
  double cost1 = lookBesidesPenalty(car, r, 1 * next); // the cost for the next lane
  double cost2 = lookBesidesPenalty(car, r, 2 * next); // the cost for the next lane

  if (cost1 >= constants::MAX_COST) {
    return constants::MAX_COST; // not safe to change lanes
  }

  if (cost1 < cost || cost2 < cost) {
    return min(cost1, cost2);
  }
  return cost + 1; // to show actual values but not selected in ties
}

double Planner::costKeepLane(const Vehicle &car, const Road &r) {
  int currentLane = car.getLane();
  auto adj = r.adjacentLanes(currentLane);
  double positionPenalty = 0.0;

  if (adj.size() == 1) {
    positionPenalty = constants::PENALTY_LANE_POSITION;
  }
  return lookAheadPenalty(car, r, currentLane) + positionPenalty;
}

double Planner::costMatchFrontSpeed(const Vehicle &car, const Road &r) {
  return constants::LOOK_AHEAD_DISTANCE - constants::SAFETY_DISTANCE;
}

double Planner::costEmergencyBrake(const Vehicle &car, const Road &r) {
  return constants::LOOK_AHEAD_DISTANCE - constants::CAR_LENGTH * 2;
}

StateGoal Planner::realizePlan(string mode, Vehicle &car, const Road &r) {
  StateGoal newGoal;
  if (mode == "KL") {
    newGoal = realizeKeepLane(car, r);
  } else if (mode == "MF") {
    newGoal = realizeMatchFront(car, r);
  } else if (mode == "EB") {
    newGoal = realizeEmergencyBrake(car, r);
  } else if (mode == "LCL") {
    newGoal = realizeChangeLeft(car, r);
  } else if (mode == "LCR") {
    newGoal = realizeChangeRight(car, r);
  } else if (mode == "ON") {
    newGoal = realizeStartEngine(car, r);
  } else {
    newGoal = realizeKeepLane(car, r); // default if wrong mode is selected.
  }
  // fix end of track condition
  if (newGoal.start_s[0] > constants::TRACKLENGTH && newGoal.end_s[0] > constants::TRACKLENGTH) {
    newGoal.start_s[0] = fmod(newGoal.start_s[0], constants::TRACKLENGTH);
    newGoal.end_s[0] = fmod(newGoal.end_s[0], constants::TRACKLENGTH);
  }
  logSelectedPlan(car, r, newGoal, mode);
  cout << logger.str();
  return newGoal;
}

StateGoal Planner::realizeKeepLane(Vehicle &car, const Road &r) {
  StateGoal goal;
  // 1. estimate needed speed
  // 2. create goal
  goal.duration = constants::KL_DURATION;
  goal.start_s = car.currentGoal.end_s;
  goal.start_d = car.currentGoal.end_d;

  goal.end_d = {car.getTargetD(car.getLane()), 0.0, 0.0}; // maybe the car is not centered so center it
  goal.end_s = endGoalFromTargetVelocity(goal, r, r.rcfg.target_speed);
  // 3. setup next update
  nextUpdateIn += (int) (goal.duration * constants::FRAMES_PER_SEC - reportedLag); // 10 points before the en
  return goal;
}

StateGoal Planner::realizeMatchFront(Vehicle &car, const Road &r) {
  StateGoal goal;
  // 1. estimate needed speed
  auto c_v = car.currentGoal.start_s[1];
  auto c_a = car.currentGoal.start_s[2];

  vector<double> frontResults = r.distanceInFront(car, car.getLane());
  double frontSpeed = frontResults[1];
  // 2. create goal
  goal.duration = constants::MF_DURATION;
  goal.start_s = car.currentGoal.end_s;
  goal.start_d = car.currentGoal.end_d;

  goal.end_d = {car.getTargetD(car.getLane()), 0.0, 0.0};
  goal.end_s = endGoalFromTargetVelocity(goal, r, frontSpeed - 5);
  // 3. setup next update
  nextUpdateIn += (int) (goal.duration * constants::FRAMES_PER_SEC - reportedLag); // 10 points before the end
  return goal;
}

StateGoal Planner::realizeChangeLeft(Vehicle &car, const Road &r) {
  // 1. estimate needed speed (keep speed)
  // 2. create goal
  StateGoal goal;
  goal.duration = constants::CL_DURATION;
  goal.start_s = car.currentGoal.end_s;
  goal.start_d = car.currentGoal.end_d;
  double delta_d = goal.start_d[0] - r.rcfg.lane_width;

  goal.end_s = endGoalFromTargetVelocity(goal, r, constants::MAX_SPEED); // keep the same speed
  goal.end_d = {delta_d, 0.0, 0.0}; // I want have zero perpendicular speed at the end
  // 4. setup next update
  nextUpdateIn += (int) (goal.duration * constants::FRAMES_PER_SEC - reportedLag); // 10 points before the end

  return goal;
}

StateGoal Planner::realizeChangeRight(Vehicle &car, const Road &r) {
  // 1. estimate needed speed (keep speed)
  // 2. create goal
  StateGoal goal;
  goal.duration = constants::CL_DURATION;
  goal.start_s = car.currentGoal.end_s;
  goal.start_d = car.currentGoal.end_d;
  double delta_d = goal.start_d[0] + r.rcfg.lane_width;

  goal.end_s = endGoalFromTargetVelocity(goal, r, constants::MAX_SPEED); // keep the same speed
  goal.end_d = {delta_d, 0.0, 0.0}; // I want have zero perpendicular speed at the end
  // 4. setup next update
  nextUpdateIn += (int) (goal.duration * constants::FRAMES_PER_SEC - reportedLag); // 10 points before the end

  return goal;
}

StateGoal Planner::realizeEmergencyBrake(const Vehicle &car, const Road &r) {
  StateGoal goal;
  // 1. estimate needed speed

  vector<double> frontResults = r.distanceInFront(car, car.getLane());
  double frontSpeed = frontResults[1];
  auto targetSpeed = frontSpeed / 2.0;

  // 2. create goal
  goal.duration = constants::MF_DURATION;
  goal.start_s = car.currentGoal.end_s;
  goal.start_d = car.currentGoal.end_d;

  auto new_s = goal.start_s[0] + targetSpeed * goal.duration;

  goal.end_d = {car.getTargetD(car.getLane()), 0.0, 0.0};
  goal.end_s = {new_s, targetSpeed, -0.8 * constants::MAX_ACCELERATION};
  // 3. setup next update
  nextUpdateIn += (int) (goal.duration * constants::FRAMES_PER_SEC - reportedLag); // 10 points before the end
  return StateGoal();
}

StateGoal Planner::realizeStartEngine(Vehicle &car, const Road &r) {
  const double target_speed = 20.0;
  const double target_s = fmod(car.s + 40.0, constants::TRACKLENGTH);

  StateGoal goal;
  goal.duration = constants::START_DURATION;
  goal.start_s = {car.s, car.speed, 0.0};
  goal.start_d = {car.d, 0.0, 0.0};
  goal.end_s = {target_s, target_speed, 0.0};
  goal.end_d = {car.getTargetD(car.getLane()), 0.0, 0.0};

  nextUpdateIn += goal.duration * constants::FRAMES_PER_SEC - reportedLag; // 10 points before the end

  return goal;
}

vector<double> Planner::endGoalFromTargetVelocity(const StateGoal &state, const Road &r, double targetVelocity) {
  double s = state.start_s[0];
  double speed = state.start_s[1];
  double acc = state.start_s[2];
  double d = state.start_d[0];

  // finding bounds for jerk , with a little slack
  double upBoundJerk = 0.8 * min((constants::MAX_ACCELERATION - acc) / state.duration, constants::MAX_JERK);
  double loBoundJerk = 0.8 * max((-constants::MAX_ACCELERATION - acc) / state.duration, -constants::MAX_JERK);

  // finding bounds for speed
  double upBound = speed + upBoundJerk * pow(state.duration, 2.) / 2.; // due to jerk
  double lowBound = speed + loBoundJerk * pow(state.duration, 2.) / 2.;
  upBound = min(upBound, r.rcfg.target_speed); // due to speed limits
  lowBound = max(lowBound, 0.0);

  targetVelocity = (targetVelocity >= upBound) ? upBound : targetVelocity; // respect bounds
  targetVelocity = (targetVelocity < lowBound) ? lowBound : targetVelocity;
  double newJerk = 2 * (targetVelocity - speed - acc * state.duration) / pow(state.duration, 2);

  double possibleDistance = speed * state.duration;
  possibleDistance += newJerk * pow(state.duration, 3) / 6.0; // !!!!important!!!!
  double newAcceleration = 0;
  double newSpeed = targetVelocity;
  double new_S = s + possibleDistance; // / curvatureFactor[0];

  return vector<double>{new_S, newSpeed, newAcceleration};
}

double Planner::lookAheadPenalty(const Vehicle &car, const Road &r, const int lane) {
  auto cl = r.distanceInFront(car, lane);
  auto distance = cl[0];
  auto speed = cl[1];
  auto mySpeed = car.currentGoal.end_s[1];

  if (distance < 2 * constants::TARGET_SPEED &&
      (distance + 2 * (speed - mySpeed)) < 2 * constants::TARGET_SPEED) {
    return constants::MAX_COST;
  }
  if (constants::LOOK_AHEAD_DISTANCE > distance) {
    return constants::LOOK_AHEAD_DISTANCE - distance;
  }
  return 0.0;
}

double Planner::lookBehindPenalty(const Vehicle &car, const Road &r, const int lane) {
  auto cl = r.distanceBehind(car, lane);
  auto distance = cl[0];
  auto speed = cl[1];
  auto mySpeed = car.currentGoal.end_s[1];

  if ((distance + constants::CL_DURATION * (speed - mySpeed)) < constants::CL_DURATION * constants::TARGET_SPEED) {
    return constants::MAX_COST;
  }
  if (constants::LOOK_BACK_DISTANCE > distance) {
    return constants::LOOK_BACK_DISTANCE - distance;
  }

  return 0.0;

}

double Planner::lookBesidesPenalty(const Vehicle &car, const Road &r, const int offset) {
  // 1. offset lane
  int cl = car.getLane() + offset; // left = -1, -2.. etc // 0 is current lane // right = +1, +2..
  // 2. check not valid lane
  if ((cl < 1) || (cl > r.rcfg.lanes)) {
    return constants::MAX_COST;
  }
  // 3. safety check
  auto safetyCheck = lookAheadPenalty(car, r, cl) + lookBehindPenalty(car, r, cl);

  if (safetyCheck >= constants::MAX_COST) {
    return constants::MAX_COST;
  }

  // 4. position penalty
  auto p_penalty = (r.adjacentLanes(cl).size() == 1) ? constants::PENALTY_LANE_POSITION : 0.0;
  // 5. total cost for driving in lane
  double cost = safetyCheck;
  cost += constants::PENALTY_LANE_CHANGE;
  cost += p_penalty;

  return cost;
}
void Planner::logSelectedPlan(const Vehicle &car, const Road &r, const StateGoal &goal, string selectedPlan) {
  vector<double> frontResults = r.distanceInFront(car, car.getLane());
  vector<double> backResults = r.distanceInFront(car, car.getLane());
  double frontDistance = frontResults[0];
  double frontSpeed = frontResults[1];

  auto b1 = r.distanceBehind(car, 1)[0];
  auto b2 = r.distanceBehind(car, 2)[0];
  auto b3 = r.distanceBehind(car, 3)[0];
  auto f1 = r.distanceInFront(car, 1)[0];
  auto f2 = r.distanceInFront(car, 2)[0];
  auto f3 = r.distanceInFront(car, 3)[0];

  logger << "******************   SELECTED PLAN : " << selectedPlan << endl;
  logger << "******************   PLAN DURATION : " << goal.duration << endl;

  logger << "LANE NO : " << car.getLane() << endl;
  logger << "FRONT SPACE: " << frontDistance << "m  BACK SPACE: " << b1 << " m" << endl;
  logger << "SPEED: " << frontSpeed << " m/s" << "ACC: " << car.acc << endl;
  logger << "S: " << car.s << "      D: " << car.d << endl;
  logger << "PLAN : ----------------------------------------------------------";
  logger << "START  S " << goal.start_s[0] << " START V  " << goal.start_s[1] << " START A " << goal.start_s[2] << endl;
  logger << "TARGET S " << goal.end_s[0] << " TARGET V  " << goal.end_s[1] << " TARGET A " << goal.end_s[2] << endl;
  logger << endl;

  logger << "*********************** LANES ANALYSIS *****************************" << endl;
  logger << " LANE 1: ---" << endl;
  logger << " -----   LOOK AHEAD PENALTY: " << lookAheadPenalty(car, r, 1) << " ( " << f1 << " )" << endl;
  logger << " -----   LOOK BEHIND PENALTY: " << lookBehindPenalty(car, r, 1) << " ( " << b1 << " )" << endl << endl;
  logger << " LANE 2: ---" << endl;
  logger << " -----   LOOK AHEAD PENALTY: " << lookAheadPenalty(car, r, 2) << " ( " << f2 << " )" << endl;
  logger << " -----   LOOK BEHIND PENALTY: " << lookBehindPenalty(car, r, 2) << " ( " << b2 << " )" << endl << endl;
  logger << " LANE 3: ---" << endl;
  logger << " -----   LOOK AHEAD PENALTY: " << lookAheadPenalty(car, r, 3) << " ( " << f3 << " )" << endl;
  logger << " -----   LOOK BEHIND PENALTY: " << lookBehindPenalty(car, r, 3) << " ( " << b3 << " )" << endl << endl;

  logger << " MY LANE: --- " << car.getLane() << endl;
  logger << " -----   LOOK BESIDES COST LEFT:  0 " << lookBesidesPenalty(car, r, 0) << endl;
  logger << " -----   LOOK BESIDES COST LEFT: -1 " << lookBesidesPenalty(car, r, -1) << endl;
  logger << " -----   LOOK BESIDES COST LEFT: -2 " << lookBesidesPenalty(car, r, -2) << endl;
  logger << endl << endl;
  logger << " -----   LOOK BESIDES COST RIGHT:  0 " << lookBesidesPenalty(car, r, 0) << endl;
  logger << " -----   LOOK BESIDES COST RIGHT:  1 " << lookBesidesPenalty(car, r, 1) << endl;
  logger << " -----   LOOK BESIDES COST RIGHT:  2 " << lookBesidesPenalty(car, r, 2) << endl;
}

//array<bool, 3> Planner::collisionDetector(const Vehicle &car, const Road &r, double planDuration) {
//  // 1. get my car position
//  auto c_s = car.currentGoal.end_s[0];
//  auto c_d = car.currentGoal.end_d[0];
//  // 2. check for adjacent lanes
//  auto lane = car.getLane();
//  auto adj = r.adjacentLanes(lane);
//  bool collision = false;
//  vector<double> dangerous_positions;
//
//  // 3. for each adjacent lane collect
//  //   a. the side they are coming from, to know if I can escape to that lane
//  //   b. the s position to later exclude those from available positions for me.
//  bool leftIsSafe = true;
//  bool rightIsSafe = true; // consider that both lane are available and free
//
//  for (size_t i = 0; i < adj.size(); i++) {
//
//    auto car_b = r.distanceBehind(car, adj[i]); // d2, v, index
//    auto car_f = r.distanceInFront(car, adj[i]); // d2, v, index
//    // check those cars and update escape availiability in left and right lanes.
//    carIsThreat(r, planDuration, c_s, c_d, dangerous_positions, car_b, leftIsSafe, rightIsSafe);
//    carIsThreat(r, planDuration, c_s, c_d, dangerous_positions, car_f, leftIsSafe, rightIsSafe);
//  } // finished collecting possible threats.
//  // now i know the threats and if i have an escape route to the left or right lanes.
//
//  if (dangerous_positions.size() == 0) {
//    // no collision threats
//    return {collision, rightIsSafe, leftIsSafe}; // crash? leftIsSafe? rightIsSafe?
//  }
//  collision = true;
//  return {collision, rightIsSafe, leftIsSafe};
//}
//
//void Planner::carIsThreat(const Road &r, double planDuration, double car_s, double car_d,
//                          vector<double> &dangerousCars, const vector<double> &other_car,
//                          bool leftIsSafe, bool rightIsSafe) const {
//  if (other_car[0] <= constants::CL_DETECTION_DIST) {
//    // 1. get car index
//    auto index = other_car[2];
//    // 2. get the state at the end of the plan duration
//    auto state_b_t = r.cars[index].getStateAt(planDuration); // state at end of plan
//    // 3. check if it goes on me
//    if (abs(state_b_t[1] - car_d) < constants::SAFETY_WIDTH && abs(state_b_t[0] - car_s) < constants::CAR_LENGTH) {
//      dangerousCars.push_back(r.cars[index].s); // its a danger
//    }
//    // 4. mark that lane as unsafe as escape route
//    if (state_b_t[1] < car_d) { leftIsSafe = false; }
//    if (state_b_t[1] > car_d) { rightIsSafe = false; }
//  }
//}

