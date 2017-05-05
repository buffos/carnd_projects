#define _USE_MATH_DEFINES

#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <cmath>
#include <string>
#include <algorithm>


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double restrictInRange(double value, double low, double high) {
	if (value > high) {
		value = high;
	}
	if (value < low) {
		value = low;
	}
	return value;
}



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::stringstream hasData(std::string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_last_of("]");
	if (found_null != std::string::npos) {
		return std::stringstream();
	}
	else if (b1 != std::string::npos && b2 != std::string::npos) {
		std::stringstream tmp = std::stringstream();
		tmp.str(s.substr(b1, b2 - b1 + 1));
		return tmp;
	}
	return std::stringstream();
}


int main(int argc, char* argv[])
{
	uWS::Hub h;
	//bool calibrationON = false;

	PID pid("streering_angle_pid");
	PID speed_pid("speed_pid");

	// std::vector<double> pid_p = { 0.09, 0.001, 2.0};
	std::vector<double> pid_p = { 0.088, 0.0, 2.0 };
	std::vector<double> speed_p = { 0.2, 0.001, 2.0 };
	std::vector<double> dp = { 0.1, 0.0001, 1.0 };

	// Initialize the pid variable.
	pid.Init(pid_p[0], pid_p[1], pid_p[2]);
	speed_pid.Init(speed_p[0], speed_p[1], speed_p[2]);

	// enable twiddling if true is passed as an argument to the command line
	if (argc == 2) {
		std::string enableTwiddling = argv[1];
		std::transform(enableTwiddling.begin(), enableTwiddling.end(), enableTwiddling.begin(), ::tolower); // to lower case
		if (enableTwiddling == "true") {
			pid.calibrationON(pid_p, dp); // enable twiddling
			// speed_pid.calibrationON(speed_p, dp);
			std::cout << "Twiddling the PID coefficients..." << std::endl;
		}
	}

	h.onMessage([&pid, &speed_pid](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		std::string reset_msg = "42[\"reset\", {}]";
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{
			auto s = hasData(std::string(data));
			if (s.str() != "") {
				auto j = json::parse(s);
				std::string event = j[0].get<std::string>();
				if (event == "telemetry") {
					// j[1] is the data JSON object
					double cte = std::stod(j[1]["cte"].get<std::string>());
					double speed = std::stod(j[1]["speed"].get<std::string>());
					double angle = std::stod(j[1]["steering_angle"].get<std::string>());
					double steer_value;
					double speed_value;
					double speed_target;
					const double speed_max = 60;
					const double speed_min = 30;
					const double speed_mid = 50;
					/*
					* TODO: Calcuate steering value here, remember the steering value is
					* [-1, 1].
					* NOTE: Feel free to play around with the throttle and speed. Maybe use
					* another PID controller to control the speed!
					*/
					pid.UpdateError(cte);
					pid.twiddle(); // twiddles if calibration is on
					steer_value = restrictInRange(pid.TotalError(), -1.0, 1.0);
					if (std::abs(steer_value) < 0.10) {
						speed_target = speed_max;
					}
					else if (0.20 > std::abs(steer_value) >= 0.10) {
						speed_target = speed_mid;
					}
					else {
						speed_target = speed_min;
					}

					double speed_error = speed - speed_target;
					speed_pid.UpdateError(speed_error);
					// speed_pid.twiddle();
					speed_value = restrictInRange(speed_pid.TotalError(), -10.0, 70.);

					// crash detection
					if (std::abs(cte) > 5.0) {
						pid.calibrationRESET();
						speed_pid.calibrationRESET();
						std::cout << "Crash Detected. Resetting....." << std::endl;
						(*ws).send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
					}


					// DEBUG
					// std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
					// std::cout << "Speed Target: " << speed_target << " Speed Error: " << speed_error << " Throttle: " << speed_pid.TotalError() << std::endl;

					json msgJson;
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = speed_value;
					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					// std::cout << msg << std::endl;
					(*ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				(*ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
		(*ws).close();
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