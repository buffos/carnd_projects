#include "planner.h"

map<string, vector<string>> Planner::next_modes = {
    {"KL", {"KL", "LCL", "LCR", "MF"}},
    {"MF", {"MF", "KL", "LCL", "LCR"}},
    {"LCL", {"KL"}},
    {"LCR", {"KL"}},
    {"OFF", {"ON"}},
    {"ON", {"KL", "LCL", "LCR", "MF"}}
};

string Planner::select_mode(const Vehicle &car, const Road &r) {
  vector<pair<double, string>> next_modes_cost;
  const vector<string> possible_modes = Planner::next_modes[car.mode]; // a vector holding all the possible next modes

  for (const string &next_mode : possible_modes) {
    double cost;
    if (next_mode == "LCL") {
      cost = costLaneChangeLeft(car, r);
    } else if (next_mode == "LCR") {
      cost = costLaneChangeRight(car, r);
    } else if (next_mode == "KL") {
      cost = costKeepLane(car, r);
    } else if (next_mode == "MF") {
      cost = costMatchFrontSpeed(car, r);
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

double Planner::costLaneChangeLeft(const Vehicle &car, const Road &r) {
  int currentLane = car.getLane();

  if (currentLane == 1) {
    return maxCost;
  }

  vector<double> frontResults = r.distanceInFront(car, currentLane - 1);
  vector<double> behindResults = r.distanceBehind(car, currentLane - 1);
  double frontDistance = frontResults[0];
  double frontSpeed = frontResults[1];
  double behindDistance = behindResults[0];
  // double behindSpeed = behindResults[1];

  if (frontDistance == 0 || behindDistance == 0) {
    return maxCost;
  }
  double costForSpace = costSpace(car, nearBuffer, frontDistance, behindDistance);
  // this is exactly zero when I have nearBuffer space both in front and behind the vehicle
  double costForSpeed = costSpeed(car, r.rcfg.target_speed, frontDistance, frontSpeed);
  return costForSpace + costForSpeed;
}

double Planner::costLaneChangeRight(const Vehicle &car, const Road &r) {
  int currentLane = car.getLane();

  if (currentLane == r.rcfg.lanes) {
    return maxCost;
  }

  vector<double> frontResults = r.distanceInFront(car, currentLane + 1);
  vector<double> behindResults = r.distanceBehind(car, currentLane + 1);
  double frontDistance = frontResults[0];
  double frontSpeed = frontResults[1];
  double behindDistance = behindResults[0];
  // double behindSpeed = behindResults[1];

  if (frontDistance == 0 || behindDistance == 0) {
    return maxCost;
  }
  double costForSpace = costSpace(car, nearBuffer, frontDistance, behindDistance);
  // this is exactly zero when I have nearBuffer space both in front and behind the vehicle
  double costForSpeed = costSpeed(car, r.rcfg.target_speed, frontDistance, frontSpeed);
  return costForSpace + costForSpeed;
}

double Planner::costKeepLane(const Vehicle &car, const Road &r) {
  int currentLane = car.getLane();
  vector<double> frontResults = r.distanceInFront(car, currentLane);
  double frontDistance = frontResults[0];
  double frontSpeed = frontResults[1];
  if (frontDistance == 0) {
    return maxCost;
  }
  double costForSpace = costSpace(car, nearBuffer, frontDistance, 0.0);
  double costForSpeed = costSpeed(car, r.rcfg.target_speed, frontDistance, frontSpeed);
  return costForSpace + costForSpeed;
}

double Planner::costMatchFrontSpeed(const Vehicle &car, const Road &r) {
  int currentLane = car.getLane();
  double slowDownCost = 0.0;

  vector<double> frontResults = r.distanceInFront(car, currentLane);
  // double frontDistance = frontResults[0];
  double frontSpeed = frontResults[1];

  if (frontSpeed < r.rcfg.target_speed
      && frontSpeed <= car.speed) { // the front car is moving slower than my target speed and slower than me.
    slowDownCost = 100 * (r.rcfg.target_speed - frontSpeed);
  }
  return slowDownCost;
}

double Planner::costSpeed(const Vehicle &car, double desiredSpeed, double freeRoadAhead, double speedCarInFront) {
  double relativeSpeed = desiredSpeed - speedCarInFront;
  if (relativeSpeed <= 0) {
    return 0.0; // no cost. the car in front is going very fast or no car in front
  } else {
    double timeToReachFrontCar = (freeRoadAhead - nearBuffer) / relativeSpeed;
    timeToReachFrontCar =
        (timeToReachFrontCar < 0) ? 0.0 : timeToReachFrontCar; // in case we are very close this gets negative
    return constants::WEIGHT_NEED_FOR_SPEED * timeToReachFrontCar;
  }
}

double Planner::costSpace(const Vehicle &car, double spaceNeeded, double spaceInFront, double spaceBehind) {
  double cost;

  if (spaceBehind == 0.0 && spaceInFront == 0.0) {
    return maxCost; // collision
  }

  if (spaceBehind == 0.0) {
    cost = spaceNeeded / spaceInFront;
  } else if (spaceInFront == 0) {
    cost = spaceNeeded / spaceBehind;
  } else {
    cost = spaceNeeded / spaceInFront + spaceNeeded / spaceBehind;
  }
  return constants::WEIGHT_NEED_FOR_SPACE * cost;
}

vector<double> Planner::endGoalFromTargetVelocity(const StateGoal &state, const Road &r, double targetVelocity) {
  double s = state.start_s[0];
  double speed = state.start_s[1];
  double acc = state.start_s[2];
  double d = state.start_d[0];

  double upBound = speed + constants::MAX_JERK * pow(state.duration, 2.) / 2.;
  double lowBound = speed - constants::MAX_JERK * pow(state.duration, 2.) / 2.;

  upBound = min(upBound, r.rcfg.target_speed); // respect jerk and target speed limit
  lowBound =
      max(lowBound, min(upBound, constants::MIN_SPEED)); // respect jerk and min speed limit (if its not violates jerk)


  targetVelocity = (targetVelocity >= upBound) ? upBound : targetVelocity; // no speeding
  targetVelocity = (targetVelocity < lowBound) ? lowBound : targetVelocity; // not going very slow

  // lets see how much we will travel with that speed
  double newJerk = 2 * (targetVelocity - speed) / pow(state.duration, 2);
  double possibleDistance = speed * state.duration + newJerk * pow(state.duration, 3) / 6.0; // !!!! important !!!!
  double newAcceleration = 0.0;
  double newSpeed = targetVelocity;
  double new_S = s + possibleDistance; // / curvatureFactor[0];

  return vector<double>{new_S, newSpeed, newAcceleration};
}

StateGoal Planner::realizePlan(string mode, Vehicle &car, const Road &r) {
  if (mode == "KL") {
    return realizeKeepLane(car, r);
  } else if (mode == "MF") {
    return realizeMatchFront(car, r);
  } else if (mode == "LCL") {
    return realizeChangeLeft(car, r);
  } else if (mode == "LCR") {
    return realizeChangeRight(car, r);
  } else if (mode == "ON") {
    return realizeStartEngine(car, r);
  } else {
    return realizeKeepLane(car, r); // default if wrong mode is selected.
  }
}

StateGoal Planner::realizeKeepLane(Vehicle &car, const Road &r) {
  StateGoal goal;
  // 1. estimate needed speed
  // 2. create goal
  goal.duration = constants::MF_DURATION;
  goal.start_s = car.currentGoal.end_s;
  goal.start_d = car.currentGoal.end_d;

  goal.end_d = {car.getTargetD(car.getLane()), 0.0, 0.0}; // maybe the car is not centered so center it
  goal.end_s = endGoalFromTargetVelocity(goal, r, r.rcfg.target_speed);
  // 3. setup next update
  nextUpdateIn += goal.duration * constants::FRAMES_PER_SEC - reportedLag; // 10 points before the en
  return goal;
}

StateGoal Planner::realizeMatchFront(Vehicle &car, const Road &r) {
  StateGoal goal;
  // 1. estimate needed speed
  vector<double> frontResults = r.distanceInFront(car, car.getLane());
  double frontDistance = frontResults[0];
  double frontSpeed = frontResults[1];

  frontSpeed = frontDistance / constants::MF_DURATION; // in case it brakes before next iteration.
  // 2. create goal
  goal.duration = constants::MF_DURATION;
  goal.start_s = car.currentGoal.end_s;
  goal.start_d = car.currentGoal.end_d;

  goal.end_d = {car.getTargetD(car.getLane()), 0.0, 0.0};
  goal.end_s = endGoalFromTargetVelocity(goal, r, frontSpeed);
  // 3. setup next update
  nextUpdateIn += goal.duration * constants::FRAMES_PER_SEC - reportedLag; // 10 points before the end

  cout << "PLAN DURATION: " << goal.duration << endl;
  cout << "car distance in front: " << frontDistance << " car speed " << frontSpeed << endl;
  cout << "my current lane is: " << car.getLane() << " and my current d is " << car.d << endl;
  cout << "current s " << car.s << " current speed  " << car.speed << " current acceleration " << car.acc << endl;
  cout << "target s " << goal.end_s[0] << " target speed  " << goal.end_s[1] << " target acceleration " << goal.end_s[2]
       << endl << endl;
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

  goal.end_s = endGoalFromTargetVelocity(goal, r, goal.start_s[1]); // keep the same speed
  goal.end_d = {delta_d, 0.0, 0.0}; // I want have zero perpendicular speed at the end
  // 4. setup next update
  nextUpdateIn += goal.duration * constants::FRAMES_PER_SEC - reportedLag; // 10 points before the end

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

  goal.end_s = endGoalFromTargetVelocity(goal, r, goal.start_s[1]); // keep the same speed
  goal.end_d = {delta_d, 0.0, 0.0}; // I want have zero perpendicular speed at the end
  // 4. setup next update
  nextUpdateIn += goal.duration * constants::FRAMES_PER_SEC - reportedLag; // 10 points before the end

  return goal;
}

StateGoal Planner::realizeStartEngine(Vehicle &car, const Road &r) {
  const double target_speed = 20.0;
  const double target_s = car.s + 40.0;

  StateGoal goal;
  goal.duration = constants::START_DURATION;
  goal.start_s = {car.s, car.speed, 0.0};
  goal.start_d = {car.d, 0.0, 0.0};
  goal.end_s = {target_s, target_speed, 0.0};
  goal.end_d = {car.getTargetD(car.getLane()), 0.0, 0.0};

  goal.printGoal();

  nextUpdateIn += goal.duration * constants::FRAMES_PER_SEC - reportedLag; // 10 points before the end

  return goal;
}