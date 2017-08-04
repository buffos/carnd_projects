#include "navigator.h"

Navigator::Navigator()
{
	// 1. initiallize logger
	if (log_file != "") {
		logger.open(log_file.c_str(), fstream::out); 
	}
	// 2. initialize road track
	road.readWayPointsFromFile(map_file);
	road.createTrackSpline();
	// 3. put car on the road.
	car.useRoadConfiguration(road.rcfg); 
}

Navigator::~Navigator()
{
	if (logger.is_open()) { logger.close(); }
}

Navigator::Navigator(string filename)
{
	map_file = filename;
	// 1. initialize road track
	road.readWayPointsFromFile(map_file);
	road.createTrackSpline();
	// 3. put car on the road.
	car.useRoadConfiguration(road.rcfg); 
}

string Navigator::operator() (const nlohmann::json &json) {
	// 1. Update data
	car.readPreviousPath(json); // reads previous path, end_path data
	if (!plan.shouldUpdate()) { return car.previousCurve.toJson(); }
	car.updateData(json, 1, true);
	road.updateData(json);
	// 2. Create new goal
	auto newMode = plan.select_mode(car, road);
	car.mode = newMode;
	cout << "NEW MODE : " << newMode << endl;
	auto newGoal = plan.realizePlan(newMode, car, road);
	car.currentGoal = newGoal;
	// 3. Generate new trajectory
	auto trajectory = tr_generator.generateTrajectory(newGoal, car, road);
	// 4. Create new discrete curve
	int numberOfPoints = (int)(road.rcfg.frames * newGoal.duration);
	auto newCurve = std::move(curveHandler.createCurve(trajectory, road, numberOfPoints));
	// 5. Merge with previous curve
	auto mergedCurve = curveHandler.appendCurve(newCurve, car.previousCurve);
	// 6. Log curves
	//logger << "OLD CURVE : " << endl;
	//car.previousCurve.toCSV(logger);
	//logger << "NEW CURVE : " << endl;
	//newCurve.toCSV(logger);
	//logger << "MERGE CURVE : " << endl;
	//mergedCurve.toCSV(logger);

	return mergedCurve.toJson();
}
