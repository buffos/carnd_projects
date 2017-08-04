#ifndef VARIOUS_STRUCTS_H
#define VARIOUS_STRUCTS_H

#include "../src/json.hpp"
#include <vector>
#include <fstream>
#include "spline.h"
#include "constants.h"

using json = nlohmann::json;

using namespace std;

struct RoadConfiguration {
	double max_s = constants::TRACKLENGTH;                               // in meters after that its a loop
	double max_speed = constants::MAX_SPEED;               // 60mph = 26.82m/s
	double target_speed = constants::TARGET_SPEED;           // 50mph = 22.352m/s
	double max_acceleration = constants::MAX_ACCELERATION; // 10m/s^2
	double max_jerk = constants::MAX_JERK;                   // 50/m/s^3
	int frames = constants::FRAMES_PER_SEC;                   // 50 frames per second
	int lanes = constants::DEFAULT_LANES;                   // 3 lanes default value
	double lane_width = constants::DEFAULT_LANE_WIDTH;       // 4m default width lane
};

struct StateGoal {
	vector<double> start_s{ 0.0, 0.0, 0.0 };
	vector<double> end_s{ 0.0, 0.0, 0.0 };
	vector<double> start_d{ 0.0, 0.0, 0.0 };
	vector<double> end_d{ 0.0, 0.0, 0.0 };
	double duration;

	StateGoal() : duration(constants::PLAN_DURATION) {}
	explicit StateGoal(double duration) : duration(duration) {}
	inline void printGoal() {
		cout << "start s: " << start_s[0] << " start vel: " << start_s[1]
			<< " start acc:" << start_s[2] << endl;
		cout << "end s: " << end_s[0] << " end vel: " << end_s[1] << " end acc:"
			<< end_s[2] << endl;
		cout << "start d: " << start_d[0] << " start vel: " << start_d[1]
			<< " start acc:" << start_d[2] << endl;
		cout << "end d: " << end_d[0] << " end vel: " << end_d[1] << " end acc:"
			<< end_d[2] << endl;
	};

	inline void printGoal(std::ofstream &log) {
		log << "start s: " << start_s[0] << " start vel: " << start_s[1]
			<< " start acc:" << start_s[2] << endl;
		log << "end s: " << end_s[0] << " end vel: " << end_s[1] << " end acc:"
			<< end_s[2] << endl;
		log << "start d: " << start_d[0] << " start vel: " << start_d[1]
			<< " start acc:" << start_d[2] << endl;
		log << "end d: " << end_d[0] << " end vel: " << end_d[1] << " end acc:"
			<< end_d[2] << endl;
	};
};

struct Trajectory {
	vector<double> s_trajectory; // polynomial coefficients
	vector<double> d_trajectory; // polynomial coefficients
	double duration;             // duration of trajectory
	double cost;                 // cost of the trajectory by the planner

	inline void printTrajectory() {
		cout << "curve cost: " << cost << endl;
		cout << "curve duration: " << duration << endl;
		cout << "s_trajectory:  " << s_trajectory[0] << " + " << s_trajectory[1]
			<< "x + " << s_trajectory[2] << "x^2 + ";
		cout << s_trajectory[3] << "x^3 + " << s_trajectory[4] << "x^4 + "
			<< s_trajectory[5] << "x^5" << endl;
		cout << "d_trajectory:  " << d_trajectory[0] << " + " << d_trajectory[1]
			<< "x + " << d_trajectory[2] << "x^2 + ";
		cout << d_trajectory[3] << "x^3 + " << d_trajectory[4] << "x^4 + "
			<< d_trajectory[5] << "x^5" << endl;
	}

	inline void printTrajectory(std::ofstream &log) {
		log << "curve cost: " << cost << endl;
		log << "curve duration: " << duration << endl;
		log << "s_trajectory:  " << s_trajectory[0] << " + " << s_trajectory[1]
			<< "x + " << s_trajectory[2] << "x^2 + ";
		log << s_trajectory[3] << "x^3 + " << s_trajectory[4] << "x^4 + "
			<< s_trajectory[5] << "x^5" << endl;
		log << "d_trajectory:  " << d_trajectory[0] << " + " << d_trajectory[1]
			<< "x + " << d_trajectory[2] << "x^2 + ";
		log << d_trajectory[3] << "x^3 + " << d_trajectory[4] << "x^4 + "
			<< d_trajectory[5] << "x^5" << endl;
	}
};

struct WayPoint {
	double x;
	double y;
	double s;
	double dx;
	double dy;

	inline WayPoint(double x, double y, double s, double dx, double dy)
		: x(x), y(y), s(s), dx(dx), dy(dy) {}
};

struct DiscreteCurve {
	vector<double> c_1; // coordinates on time parameter
	vector<double> c_2;
	int coordinateSystem; // 1 = XY,  2 = Frenet
	double timestep;      // the parameter-axis of those 2 parametric curves

	DiscreteCurve() {
		coordinateSystem = 2;
		timestep = 1 / constants::FRAMES_PER_SEC;
	}
	DiscreteCurve(vector<double> s, vector<double> d) {
		coordinateSystem = 2;
		timestep = 1 / constants::FRAMES_PER_SEC;
		c_1 = s;
		c_2 = d;
	};

	inline int size() const { return c_1.size(); }

	inline string toJson() {
		json msgJson;

		msgJson["next_x"] = c_1;
		msgJson["next_y"] = c_2;

		return msgJson.dump();
	}

	inline void printCurve()
	{
		string x;
		string y;
		if (coordinateSystem == 1)
		{
			cout << "Curve in XY coordinates " << endl;
			x = " x: ";
			y = " y: ";
		}
		else
		{
			cout << "Curve in Frenet coordinates " << endl;
			x = " s: ";
			y = " d: ";
		}
		cout << "timestep : " << timestep << endl;
		for (unsigned int i = 0; i < c_1.size(); i++)
		{
			cout << "i: " << i << x << c_1[i] << y << c_2[i] << endl;
		}
	}

	inline void printCurve(std::ofstream &log)
	{
		string x;
		string y;
		if (coordinateSystem == 1)
		{
			log << "Curve in XY coordinates " << endl;
			x = " x: ";
			y = " y: ";
		}
		else
		{
			log << "Curve in Frenet coordinates " << endl;
			x = " s: ";
			y = " d: ";
		}
		log << "timestep : " << timestep << endl;
		for (unsigned int i = 0; i < c_1.size(); i++)
		{
			log << "i: " << i << x << c_1[i] << y << c_2[i] << endl;
		}
	}

	inline void toCSV(std::ofstream &log)
	{
		for (unsigned int i = 0; i < c_1.size(); i++)
		{
			log << i << "," << "," << c_1[i] << "," << c_2[i] << endl;
		}
	}
};

struct Splines {
	tk::spline x;
	tk::spline y;
	tk::spline dx;
	tk::spline dy;

	double start_s = 0.0;
	double end_s = constants::TRACKLENGTH;
};
#endif //! VARIOUS_STRUCTS_H