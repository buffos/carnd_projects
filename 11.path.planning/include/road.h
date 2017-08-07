/**
 * @file road.h
 * @brief Definition of the Road object and its methods
 *
 * *
 * @author  Kostas Oreopoulos
 */
#ifndef ROAD_H
#define ROAD_H

#include "json.hpp"
#include <vector>
#include "vehicle.h"
#include "tools.h"
#include "various_structs.h"
#include "spline.h"

using json = nlohmann::json;
using namespace std;

struct Road {
  const RoadConfiguration rcfg;
  vector<Vehicle> cars;
  vector<WayPoint> wpts;
  Splines track_spline;

  inline Road() {}
  /**
   * Reaad a JSON object containing sensor fusion data about
   * the cars on that road in our traffic direction and populate
   * the cars vector
   * @param j  The JSON object
   * @param index The default position to read from (its one) and  can be omitted for our project
   */
  void updateData(const json j, int index = 1);
  /**
   * Read from a text file the coordinate of way points,
   * that roughly describe the inner (left lane) outline
   * of the road track. It then goes on to construct a more precise
   * representation of the road.
   * The WAY-POINTS contains X,Y,S,NX,NY , where Nxy is the vector
   * normal to the path at that point
   * @param filename A string containing the path (relative or absolute)
   */
  void readWayPointsFromFile(string filename);
  /**
   * This function is called from readWayPointsFromFile
   * and it construct splines that cover the whole track
   * and associate the s-param with X,Y,NX,NY
   */
  void createTrackSpline();
/**
 * For a given lane return a vector with the number(s) (order from
 * left to right) of the lanes adjacent to the lane
 * @param lane The lane we are interested in
 * @return The vector containing the adjacent lanes
 */
  vector<int> adjacentLanes(int lane) const;
  /**
   * For the car we are interested in, the function tells us
   * the nearest (in the s-sense) vehicle travelling at the
   * lane given in FRONT of the car. Info about position and speed are given
   * @param car  The car we are interested in
   * @param lane  The lane we want to search
   * @return A vector containing {Distance , Velocity , Index} (in the vehicles list)
   */
  vector<double> distanceInFront(const Vehicle &car, int lane) const;
  /**
   * For the car we are interested in, the function tells us
   * the nearest (in the s-sense) vehicle travelling at the
   * lane given BEHIND the car. Info about position and speed are given
   * @param car  The car we are interested in
   * @param lane  The lane we want to search
   * @return A vector containing {Distance , Velocity , Index} (in the vehicles list)
   */
  vector<double> distanceBehind(const Vehicle &car, int lane) const;
  /**
   * It return the distance of the closest vehicle at (s,d) coordinates
   * after a certain amount of time based on the sensor fusion date.
   * @param s  The s-coordinate of the point we are interested (Frenet Coordinates)
   * @param d  The d-coordinate of the point we are interested (Frenet Coordinates)
   * @param time The time (in the future) in seconds we are interested for a prediction
   * @return The distance of the closest vehicle to the s,d position.
   */
  double closestVehicleAt(double s, double d, double time);
/**
 * Transforms the (s,d) point from Frenet to Cartesian coordinates
 * with the help of the spline representation stored in our Road Object
 * @param s The s-coordinate
 * @param d The d-coordinate
 * @return  X,Y coordinates
 */
  vector<double> toXY(double s, double d) const;
  /**
   * An experimental function to estimate the curvature of the road we are
   * travelling at s given position
   * @param s The s-coordinate
   * @param d The d-coordinate
   * @param evaluationPoints How many points should we split the curve path
   * @param scanningDistance The length (in s-coordinates) of the path we will divide
   * @return The ratio of the actual distance in XY coordinate of the arc we travel to the s-distance
   */
  vector<double> curvatureFactor(double s, double d, const int evaluationPoints, double scanningDistance) const;
  /**
   * At a given s-coordinate, calculate the orientation (yaw) of the road lane
   * based on the tangent of the road at the point
   * @param s The s-coordinate
   * @return  The angle (in rads) of the orientation of the lane
   */
  double orientation(double s);
};

#endif // !ROAD_H