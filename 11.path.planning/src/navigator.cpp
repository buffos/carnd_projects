/**
 * @file navigator.cpp
 * @brief Implementation of the methods of Navigator
 *
 * The basic function is the () operator that initiates the planning cycle
 *
 * @author  Kostas Oreopoulos
 */

#include "navigator.h"

Navigator::Navigator() : tr_generator() {
  // 1. initialize logger
  if (!log_file.empty()) {
    logger.open(log_file.c_str(), fstream::out);
  }
  // 2. initialize road track
  road.readWayPointsFromFile(map_file);
  road.createTrackSpline();
  // 3. put car on the road.
  car.useRoadConfiguration(road.rcfg);
}

Navigator::~Navigator() {
  if (logger.is_open()) { logger.close(); }
}

Navigator::Navigator(const string &filename) : tr_generator() {
  map_file = filename;
  // 1. initialize road track
  road.readWayPointsFromFile(map_file);
  road.createTrackSpline();
  // 3. put car on the road.
  car.useRoadConfiguration(road.rcfg);
}

string Navigator::operator()(const nlohmann::json &json) {
  // 1. Update or pass
  car.readPreviousPath(json); // reads previous path, end_path data
  if (!plan.shouldUpdate()) { return car.previousCurve.toJson(); }

  //  2. Update data and calculate local data
  car.updateData(json, 1, true);
  car.updateLocalData(road.orientation(car.s));
  plan.reportedLag = car.lag();
  road.updateData(json);

  // 3. Create new goal
  auto newMode = plan.select_mode(car, road);
  car.mode = newMode;
  auto newGoal = plan.realizePlan(newMode, car, road);

  // 4. Generate new trajectory
  auto trajectory = tr_generator.generateTrajectory(newGoal, car, road);

  // 5. Keep the final plan for next starting state
  car.currentGoal = trajectory.goal;

  // 6. Create new discrete curve
  auto numberOfPoints = (int) (road.rcfg.frames * newGoal.duration);
  auto newCurve = std::move(curveHandler.createCurve(trajectory, road, numberOfPoints));

  // 7. Merge with previous curve
  auto mergedCurve = curveHandler.appendCurve(newCurve, car.previousCurve);

  return mergedCurve.toJson();
}
