#include "json.hpp"

#include <uWS/uWS.h>
#include <fstream>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "tools.h"
#include "vehicle.h"
#include "road.h"
#include "planner.h"
#include "trajectoryGenerator.h"
#include "discreteCurves.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos)
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}



int main()
{
	uWS::Hub h;

	// const string map_file_ = "../data/highway_map.csv"; // Waypoint map to read from
	const string map_file_ = "C:/Users/buffo/Code/python/prj - selfDrivingCars/carnd-project/11.path.planning/data/highway_map.csv"; // Waypoint map to read from
	const double max_s = 6945.554;																									 // The max s value before wrapping around the track back to 0
	string log_file = "I:/logger.txt";
	ofstream out_log(log_file.c_str(), fstream::out);

	Vehicle car;
	Road road;
	Planner plan;
	TrajectoryGenerator tr_generator;
	CurveHandler curveHandler;

	plan.planDuration = 15.0; // sec
	tr_generator.planDuration = 15.0; // in sec

	road.readWayPointsFromFile(map_file_);
	car.useRoadConfiguration(road.rcfg); // tell the car to use the configuration of road (basically for

	h.onMessage([&car, &road, &plan, &tr_generator, &curveHandler, &out_log](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message. The 2 signifies a websocket event
		// auto s_data = string(data).substr(0, length); cout << s_data << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{

			auto s = hasData(data);

			if (s != "")
			{
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry")
				{
					string msg;
					DiscreteCurve newCurve;

					car.readPreviousPath(j);											   // reads previous path, end_path data
					if (car.previousCurve.c_1.size() > 100)
					{
						newCurve = std::move(car.previousCurve);
						
					}
					else
					{
						car.updateData(j, 1, true);											   // localization Data from json object at position 1 (main car). YAW is in degrees so we must convert.
						road.updateData(j);													   // Sensor Fusion Data, a list of all other cars on the same side of the road.
						car.printVehicle(out_log);
						auto newMode = plan.select_mode(car, road);							   // select a new best state for the car on the road
						// auto newGoal = plan.realizePlan(newMode, car, road);				   // apply the plan and get a new goal for the trajectory generator
						auto newGoal = plan.realizePlan("MF", car, road);
						newGoal.printGoal(out_log);
						auto trajectory = tr_generator.generateTrajectory(newGoal, car, road); // generate a new trajectory
						trajectory.printTrajectory(out_log);
						newCurve = std::move(curveHandler.createCurveFromCoefficientsInXY(trajectory,  road.rcfg.frames * plan.planDuration, road.wpts));
						// create a new curve and use 10 of the previous Curve points
						out_log << "PREV CURVE " << endl;
						car.previousCurve.printCurve(out_log);
						out_log << "NEW CURVE " << endl;
						newCurve.printCurve(out_log);
						//mergedCurve = std::move(curveHandler.mergeCurves(newCurve, car.previousCurve));
						//out_log << "MERGED CURVE " << endl;
						//mergedCurve.printCurve(out_log);
					}

					 msg = "42[\"control\"," + newCurve.toJson() + "]";

					
					ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					// this_thread::sleep_for(chrono::milliseconds(500));
				}
			}
			else
			{
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
		ws->close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen("0.0.0.0", port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}