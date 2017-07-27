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

	const string map_file_ = "../data/highway_map.csv"; // Waypoint map to read from
	const double max_s = 6945.554;						// The max s value before wrapping around the track back to 0

	Vehicle car;
	Road road;
	Planner plan;
	TrajectoryGenerator tr_generator;
	CurveHandler curveHandler;

	road.readWayPointsFromFile(map_file_);
	car.useRoadConfiguration(road.rcfg); // tell the car to use the configuration of road (basically for

	h.onMessage([&car, &road, &plan, &tr_generator, &curveHandler](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
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
					car.updateData(j, 1);												   // localization Data from json object at position 1 (main car).
					road.updateData(j);													   // Sensor Fusion Data, a list of all other cars on the same side of the road.
					car.readPreviousPath(j);											   // reads previous path, end_path data
					auto newMode = plan.select_mode(car, road);							   // select a new best state for the car on the road
					auto newGoal = plan.realizePlan(newMode, car, road);				   // apply the plan and get a new goal for the trajectory generator
					auto trajectory = tr_generator.generateTrajectory(newGoal, car, road); // generate a new trajectory
					auto newCurve = std::move(curveHandler.createCurveFromCoefficientsInXY(trajectory, road.rcfg.frames, road.wpts));
					auto mergedCurve = std::move(curveHandler.mergeCurves(newCurve, car.previousCurve));

					string msg = "42[\"control\"," + mergedCurve.toJson() + "]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
	if (h.listen(port))
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
