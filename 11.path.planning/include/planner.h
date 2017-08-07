/**
 * @file planner.h
 * @brief The behavior planner: A FSM that decides the next position and action of the car
 *
 * The behavior planner is implemented as Finite State Machine.
 * For each state it evaluates the actions, quantizes them and
 * select the best one to execute. It also creates a goal state
 * in the (position, velocity, acceleration space) , containing
 * both a start and end state, that passes to the trajectory generator
 * to create a smooth path from start to end goal
 *
 * @todo Detect imminent collisions from other vehicles running to my car.
 * @todo Expand solution to (position, velocity, acceleration, jerk) space.
 * This will help the generator play with jerk and not brake the continuity
 * of the plan
 *
 * @author  Kostas Oreopoulos
 */
#ifndef PLANNER_H
#define PLANNER_H

#include <map>
#include <vector>
#include <string>
#include "various_structs.h"
#include "vehicle.h"
#include "road.h"
#include "constants.h"
#include <sstream>

using namespace std;

struct Planner {
  int nextUpdateIn = 0; ///< A counter that counts down in how many cycles the planner should update the plan
  double reportedLag = 0; ///< this stores the lag information received by the car
  std::ostringstream logger; ///< a convenience string stream to help with printing information

  static map<string, vector<string>> next_modes; ///< a map that contains the next possible states for every current

  /**
   * It returns true if its "time" to update.
   * Time is defines in how many points are left in the current
   * curve that guides the vehicle
   * @return True if its time to update
   */
  bool shouldUpdate() {
    if (nextUpdateIn <= constants::UPDATE_WHEN) { return true; }
    else {
      nextUpdateIn--;
      return false;
    }
  }

  /**
   * For the current mode the car is in, it goes through all the possible
   * next states, evaluates them and select the best
   * @param car The car object that the planning is made for
   * @param r The road object the car is driving on
   * @return A string with the mode that is selected as best
   */
  string select_mode(const Vehicle &car, const Road &r);
  /**
   * The cost of changing lanes
   * @param car : The car object that the planning is made for
   * @param r : The road object the car is driving on
   * @param left : True when it evaluates a CL to the left. False to the right
   * @return The cost of the action
   */
  double costLaneChange(const Vehicle &car, const Road &r, bool left);
  /**
   * The cost of staying in the lane
   * @param car : The car object that the planning is made for
   * @param r : The road object the car is driving on
   * @return The cost of the action
   */
  double costKeepLane(const Vehicle &car, const Road &r);
  /**
   * The cost of matching the front car's speed
   * @param car : The car object that the planning is made for
   * @param r : The road object the car is driving on
   * @return The cost of the action
   */
  double costMatchFrontSpeed(const Vehicle &car, const Road &r);
  /**
   * The cost of Emergency braking
   * @param car : The car object that the planning is made for
   * @param r : The road object the car is driving on
   * @return The cost of the action
   */
  double costEmergencyBrake(const Vehicle &car, const Road &r);

  /**
   * It take the mode that was selected and the planner has to
   * generate a goal for it. It selects the correct function to
   * accomplish the task
   * @param mode : The string containing the selected action
   * @param r : The road object the car is driving on
   * @return  A State goal object containing the start and end states of the action
   */
  StateGoal realizePlan(string mode, Vehicle &car, const Road &r);
  /**
   * Generates the goal for the Keep Lane Action
   * @param mode : The string containing the selected action
   * @param r : The road object the car is driving on
   * @return  A State goal object containing the start and end states of the action
   */
  StateGoal realizeKeepLane(Vehicle &car, const Road &r);
  /**
   * Generates the goal for the Match Front Action
   * @param mode : The string containing the selected action
   * @param r : The road object the car is driving on
   * @return  A State goal object containing the start and end states of the action
   */
  StateGoal realizeMatchFront(Vehicle &car, const Road &r);
  /**
   * Generates the goal for the Change Lane to the Left Action
   * @param mode : The string containing the selected action
   * @param r : The road object the car is driving on
   * @return  A State goal object containing the start and end states of the action
   */
  StateGoal realizeChangeLeft(Vehicle &car, const Road &r);
  /**
   * Generates the goal for the Change Lane to the Right Action
   * @param mode : The string containing the selected action
   * @param r : The road object the car is driving on
   * @return  A State goal object containing the start and end states of the action
   */
  StateGoal realizeChangeRight(Vehicle &car, const Road &r);
  /**
   * Generates the goal for Starting Engine action
   * @param mode : The string containing the selected action
   * @param r : The road object the car is driving on
   * @return  A State goal object containing the start and end states of the action
   */
  StateGoal realizeStartEngine(Vehicle &car, const Road &r);
  /**
 * Generates the goal for the Emergency action
 * @param mode : The string containing the selected action
 * @param r : The road object the car is driving on
 * @return  A State goal object containing the start and end states of the action
 */
  StateGoal realizeEmergencyBrake(const Vehicle &car, const Road &r);

  /**
   * How close it the vehicle in front, translated into cost
   * @param car : The car object that the planning is made for
   * @param r : The road object the car is driving on
   * @param lane : The lane we want to take information for
   * @return  A value describing how costly it is to be at that lane, at the s position of our car
   */
  double lookAheadPenalty(const Vehicle &car, const Road &r, const int lane);
  /**
 * How close it the vehicle in behind, translated into cost
 * @param car : The car object that the planning is made for
 * @param r : The road object the car is driving on
 * @param lane : The lane we want to take information for
 * @return  A value describing how costly it is to be at that lane, at the s position of our car
 */
  double lookBehindPenalty(const Vehicle &car, const Road &r, const int lane);
  /**
   * Creates a combined cost for both front and behind traffic along with other penalties
   * @param car : The car object that the planning is made for
   * @param r : The road object the car is driving on
   * @param offset Positive or negative offset from the current lane the car is moving
   * @return A cost of how good or bad it is for the car to be driving in that lane
   */
  double lookBesidesPenalty(const Vehicle &car, const Road &r, const int offset);
/**
 * The basic function that creates the end goals. It applies the restriction to
 * velocity, acceleration and jerk, decides first on what the jerk should be and then builds the final
 * end state. The basic limitation is that the acceleration component is set to zero
 * @param state  A StateGoal object that was created by the realizeXYPlan, that is completed by this function
   * @param r : The road object the car is driving on
 * @param targetVelocity The velocity the action has decided that is best for the car
 * @return A vector containing the end state for the s coordinate (position, velocity, acceleration)
 */
  vector<double> endGoalFromTargetVelocity(const StateGoal &state, const Road &r, double targetVelocity);
/**
 * Printing function mainly for debugging
 * @param car
 * @param r
 * @param goal
 * @param selectedPlan
 */
  void logSelectedPlan(const Vehicle &car, const Road &r, const StateGoal &goal, string selectedPlan);
};

#endif // !PLANNER_H