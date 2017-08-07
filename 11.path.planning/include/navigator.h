/**
 * @file navigator.h
 * @brief Struct that unifies and coordinates the navigation of the car
 *
 * It contains instances of the car, road, the planner ,
 * trajectory generator and the discrete curve generator
 *
 * @author  Kostas Oreopoulos
 */
#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "json.hpp"
#include "tools.h"
#include "vehicle.h"
#include "road.h"
#include "planner.h"
#include "trajectoryGenerator.h"
#include "discreteCurves.h"
#include "constants.h"

struct Navigator {
  string map_file = R"(../data/highway_map.csv)";
  string log_file = "";
  Vehicle car;
  Road road;
  Planner plan;
  TrajectoryGenerator tr_generator;
  CurveHandler curveHandler;
  ofstream logger;

  Navigator();
  Navigator(const string &filename);
  ~Navigator();

  /**
   * The only function of the Navigator class it the oveloading
   * of the () operator. It takes as argument the json passed by the simulator
   * and executes a complete planning cycle. It returns the expected way points in
   * string format to the simulator
   *
   * @param json The json object passed by the simulator to the program
   * @return  A string containing the coordinates of the curve for the car to follow.
   */
  string operator()(const nlohmann::json &json);

};

#endif