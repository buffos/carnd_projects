
#include <iostream>
#include "Eigen/Dense"
#include <vector>
#include "ukf.h"
#include "measurement_package.h"
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <iomanip>  
#include "cxxopts.hpp"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

bool useOnlyRadar = false;
bool useOnlyLidar = false;
string inputFilename = "";
string outputFilename = "";
const string delimiter = "\t";

void parseCommandLine(int argc, char *argv[]) {
	try {
		cxxopts::Options options(argv[0], " - Unscented Kalman Filter: \n"
			"Sensor data fusion.\n"
			"Input and Output files are required");

		options.add_options()
			("h,help", "Print help")
			("i,input", "Input File", cxxopts::value<std::string>())
			("o,output", "Output file", cxxopts::value<std::string>())
			("r,radar", "use only radar data", cxxopts::value<bool>(useOnlyRadar))
			("l,lidar", "use only lidar data", cxxopts::value<bool>(useOnlyLidar));

		vector<string> positional_arguments = { "input", "output" };
		options.parse_positional(positional_arguments);

		options.parse(argc, argv);

		if (options.count("help")) {
			cout << options.help({ "", "Group" }) << endl;
			exit(EXIT_SUCCESS);
		}

		if (options.count("input") == 0) {
			cout << "An Input File is required." << endl;
			exit(EXIT_FAILURE);
		}

		if (options.count("output") == 0) {
			cout << "An Output File is required." << endl;
			exit(EXIT_FAILURE);
		}

		inputFilename = options["input"].as<string>();
		outputFilename = options["output"].as<string>();

	}
	catch (const cxxopts::OptionException &e) {
		cout << "error parsing options: " << e.what() << endl;
		exit(EXIT_FAILURE);
	}
}


void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {

  parseCommandLine(argc, argv);

  ifstream in_file_(inputFilename.c_str(), ifstream::in);
  ofstream out_file_(outputFilename.c_str(), ofstream::out);

  check_files(in_file_, inputFilename, out_file_, outputFilename);

  /**********************************************
   *  Set Measurements                          *
   **********************************************/

  vector<MeasurementPackage> measurement_pack_list;
  vector<VectorXd> estimation_list, ground_truth_list;
  string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {
    string sensor_type;
    MeasurementPackage meas_package;
    istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;

	if (useOnlyRadar && sensor_type.compare("L") == 0) {
		continue;
	}
	else if (useOnlyLidar && sensor_type.compare("R") == 0) {
		continue;
	}

    if (sensor_type.compare("L") == 0) {
      // laser measurement

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float px;
      float py;
      iss >> px;
      iss >> py;
      meas_package.raw_measurements_ << px, py;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    } else if (sensor_type.compare("R") == 0) {
      // radar measurement

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float theta;
      float ro_dot;
      iss >> ro;
      iss >> theta;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, theta, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

	// read the ground truth data for RMSE calculation
	float px, py, vx, vy;
	iss >> px >> py >> vx >> vy;
	VectorXd g(4);
	g << px, py, vx, vy;
	ground_truth_list.push_back(g);
  }

  // Create a UKF instance
  UKF ukf;

  size_t number_of_measurements = measurement_pack_list.size();
  assert(ground_truth_list.size() == measurement_pack_list.size());

  // start filtering from the second frame (the speed is unknown in the first
  // frame)
  for (size_t k = 0; k < number_of_measurements; ++k) {
    // Call the UKF-based fusion
    ukf.ProcessMeasurement(measurement_pack_list[k]);

	const double x = ukf.x_(0);
	const double y = ukf.x_(1);
	const double v = ukf.x_(2);
	const double yaw = ukf.x_(3);
	const double yaw_d = ukf.x_(4);

	if (k == 0) {
		out_file_ << std::left << std::setw(15) << "positionX" << delimiter;
		out_file_ << std::left << std::setw(15) << "positionY" << delimiter;
		out_file_ << std::left << std::setw(15) << "velocity" << delimiter;
		out_file_ << std::left << std::setw(15) << "yaw" << delimiter;
		out_file_ << std::left << std::setw(15) << "yaw_dot" << delimiter;
		out_file_ << std::left << std::setw(15) << "measurementX" << delimiter;
		out_file_ << std::left << std::setw(15) << "measurementY" << delimiter;
		out_file_ << std::left << std::setw(15) << "sensor type" << delimiter;
		out_file_ << std::left << std::setw(15) << "NIS" << "\n";
	}

    // output the estimation
	out_file_ << std::left << std::setw(15) << x << delimiter;
	out_file_ << std::left << std::setw(15) << y << delimiter;
	out_file_ << std::left << std::setw(15) << v << delimiter;
	out_file_ << std::left << std::setw(15) << yaw << delimiter;
	out_file_ << std::left << std::setw(15) << yaw_d << delimiter;

	// to compare with ground truth
	const double vx = v*cos(yaw);
	const double vy = v*sin(yaw);
	VectorXd estimation(4);
	estimation << x, y, vx, vy;
	estimation_list.push_back(estimation);

    // output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      // output the estimation

      // p1 - meas
      out_file_ << std::left << std::setw(15) << measurement_pack_list[k].raw_measurements_(0) << delimiter;

      // p2 - meas
      out_file_ << std::left << std::setw(15) << measurement_pack_list[k].raw_measurements_(1) << delimiter;
	  out_file_ << std::left << std::setw(15) << "L" << delimiter; // Sensor Type
	  out_file_ << std::left << std::setw(15) << ukf.NIS_laser_ << delimiter;  // NIS for laser

    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      double ro = measurement_pack_list[k].raw_measurements_(0);
	  double phi = measurement_pack_list[k].raw_measurements_(1);
      out_file_ << std::left << std::setw(15) << ro * cos(phi) << delimiter; // p1_meas
      out_file_ << std::left << std::setw(15) << ro * sin(phi) << delimiter; // p2_meas
	  out_file_ << std::left << std::setw(15) << "R" << delimiter; // Sensor Type
	  out_file_ << std::left << std::setw(15) << ukf.NIS_radar_ << delimiter;  // NIS for radar
    }

    out_file_ << "\n";
  }

  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  const VectorXd rmse = ukf.CalculateRMSE(estimation_list, ground_truth_list);
  std::cout << "RMSE:" << endl << rmse << endl;

  cout << "Done!" << endl;
  return 0;
}
