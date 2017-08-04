#include "json.hpp"
#include "navigator.h"
#include <uWS/uWS.h>
#include <fstream>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

int main() {
	uWS::Hub h;
	Navigator navigator;
	// const string map_file_ = "../data/highway_map.csv";

	auto handler = [&](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means
		// there's a websocket message event.
		// The 4 signifies a websocket message. 
		// The 2 signifies a websocket event
		// auto s_data = string(data).substr(0, length);
		// cout << s_data << endl;
		if (length <= 2 || data[0] != '4' || data[1] != '2') {
			return;
		}

		auto s = hasData(data);

		if (s == "") {
			// Manual driving
			std::string msg = "42[\"manual\",{}]";
			ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			return;
		}

		auto j = json::parse(s);
		string event = j[0].get<string>();

		if (event != "telemetry") {
			return;
		}


		auto msg = "42[\"control\"," + navigator(j) + "]";

		ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	};

	h.onMessage(handler);

	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
		size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		}
		else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});


	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
		char *message, size_t length) {
		ws->close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen("0.0.0.0", port)) {
		std::cout << "Listening to port " << port << std::endl;
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}