/**
 * @file trajectoryGenerator.cpp
 * @brief Implementation of the Trajectory Generator class
 *
 * @author  Kostas Oreopoulos
 */
#include "trajectoryGenerator.h"
#include "polynomial.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/// helper functions
TrajectoryGenerator::TrajectoryGenerator() {
  this->numberOfSamples = constants::SAMPLE_TRAJECTORIES;
}

Trajectory TrajectoryGenerator::generateTrajectory(StateGoal &s, Vehicle &car, Road &r) {
  auto planDuration = s.duration;
  auto timestep = 1.0 / constants::FRAMES_PER_SEC;
  double lowTimeBound = planDuration - 0 * timestep;
  double highTimeBound = planDuration + 0 * timestep;

  vector<StateGoal> newGoals = std::move(perturbGoal(s, r));
  vector<Trajectory> newTrajectories;

  //for all plan duration times and all perturbations
  //of the original goal I create all possible trajectories
  for (double time = lowTimeBound; time <= highTimeBound; time += timestep) {
    for (auto &goal : newGoals) {
      Trajectory tr;
      tr.s_trajectory = jmt(goal, time, 1);
      tr.d_trajectory = jmt(goal, time, 2);
      tr.duration = time;
      tr.cost = totalCost(tr, s, car, r); // find cost of trajectory
      tr.goal = goal; // keep the new pertubed goal to inform the car about it.
      newTrajectories.push_back(tr);
    }
  }

  // sort the with increasing order based on cost
  std::sort(newTrajectories.begin(), newTrajectories.end(), [](Trajectory &i, Trajectory &j) -> bool {
    return i.cost < j.cost;
  });

  // now we evaluate the generated trajectories and select the best trajectory
  // from the sorted array of trajectories
  return newTrajectories[0];
}

/// Generate perturbed goals from the one the planner initially selected
/// The Trajectory generator will create JMT curves and evaluate them
/// Selecting the best one fitting the situation.
vector<StateGoal> TrajectoryGenerator::perturbGoal(StateGoal &s, Road &r) {
  vector<StateGoal> newGoals;
  std::default_random_engine generator; // to generate the random samples

  std::normal_distribution<double> distribute_values(0.0, sigma);

  for (int i = 0; i < numberOfSamples; i++) {
    StateGoal newStateGoal;
    double factor = distribute_values(generator);
    newStateGoal.start_s = s.start_s;
    newStateGoal.start_d = s.start_d;
    newStateGoal.end_s = {s.end_s[0] + s.end_s[1] * s.duration * factor,
                          s.end_s[1] + s.end_s[1] * factor,
                          s.end_s[2] + s.end_s[2] * factor};
    newStateGoal.end_d = {s.end_d[0], s.end_d[1], s.end_d[2]};
    newGoals.push_back(newStateGoal);
  }
  return newGoals;
}

vector<double> TrajectoryGenerator::jmt(StateGoal &s, double t, int s_or_d) {
  MatrixXd A = MatrixXd(3, 3);
  VectorXd B = MatrixXd(3, 1);

  vector<double> start;
  vector<double> end;
  if (s_or_d == 1) {
    start = s.start_s;
    end = s.end_s;
  } else {
    start = s.start_d;
    end = s.end_d;
  }

  double t2 = t * t;
  double t3 = t * t2;
  double t4 = t * t3;
  double t5 = t * t4;

  A << t3, t4, t5,
      3 * t2, 4 * t3, 5 * t4,
      6 * t, 12 * t2, 20 * t3;

  B << end[0] - (start[0] + start[1] * t + 0.5 * start[2] * t2),
      end[1] - (start[1] + start[2] * t),
      end[2] - start[2];

  VectorXd C = A.inverse() * B;

  double a0 = start[0];
  double a1 = start[1];
  double a2 = start[2] * 0.5;
  double a3 = C[0];
  double a4 = C[1];
  double a5 = C[2];

  vector<double> result = {a0, a1, a2, a3, a4, a5};

  return result;
}

/// cost because the trajectory is shorter or longer than the desired planDuration
double TrajectoryGenerator::timeDifferenceCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r) {
  double x = abs(s.duration - tr.duration) / s.duration;
  return logistic(x);
}

/// cost because the s coordinate and derivatives are not as the goal
double TrajectoryGenerator::s_DifferenceCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r) {
  Polynomial p(tr.s_trajectory); // create a polynomial based on the trajectory coefficients
  double actual_s_s = p.evalAt(tr.duration, 0);
  double actual_s_v = p.evalAt(tr.duration, 1);
  double actual_s_a = p.evalAt(tr.duration, 2);

  // compare with the original goal from planner
  double expected_s_s = s.end_s[0];
  double expected_s_v = s.end_s[1];
  double expected_s_a = s.end_s[2];

  double cost = 0.0;
  cost += logistic(abs(actual_s_s - expected_s_s) / sigma);
  cost += logistic(abs(actual_s_v - expected_s_v) / sigma);
  cost += logistic(abs(actual_s_a - expected_s_a) / sigma);

  return cost;
}

/// cost because the d coordinate and derivatives are not as the goal
double TrajectoryGenerator::d_DifferenceCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r) {
  Polynomial p(tr.s_trajectory); // create a polynomial based on the trajectory coefficients
  double actual_d_s = p.evalAt(tr.duration, 0);
  double actual_d_v = p.evalAt(tr.duration, 1);
  double actual_d_a = p.evalAt(tr.duration, 2);

  // compare with the original goal from planner
  double expected_d_s = s.end_d[0];
  double expected_d_v = s.end_d[1];
  double expected_d_a = s.end_d[2];

  double cost = 0.0;
  cost += logistic(abs(actual_d_s - expected_d_s) / sigma);
  cost += logistic(abs(actual_d_v - expected_d_v) / sigma);
  cost += logistic(abs(actual_d_a - expected_d_a) / sigma);

  return cost;
}

/// this will calculate the closest the vehicle will ever come in the trajectory with another vehicle
double TrajectoryGenerator::collisionCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r) {
  Polynomial p_s(tr.s_trajectory);
  Polynomial p_d(tr.d_trajectory);

  double dt = 1.0 / constants::FRAMES_PER_SEC;
  // split time in the number of frames i have and check the closest vehicle at that time
  for (int i = 0; i < constants::FRAMES_PER_SEC * tr.duration; i++) {
    double s_at_time = p_s.evalAt(i * dt, 0);
    double d_at_time = p_d.evalAt(i * dt, 0);
    for (auto &o_car : r.cars) {
      auto other = o_car.getStateAt(i * dt);
      auto diff_s = s_at_time - other[0];
      auto diff_d = d_at_time - other[1];
      auto d2 = pow(diff_d, 2) + pow(diff_s, 2);
      if (d2 <= constants::SAFETY_DISTANCE) {
        return 1.0;
      }
    }
  }
  return 0.0;
}

double TrajectoryGenerator::bufferCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r) {
  Polynomial p_s(tr.s_trajectory);
  Polynomial p_d(tr.d_trajectory);

  double d_closest = std::numeric_limits<double>::max();
  double dt = 1.0 / constants::FRAMES_PER_SEC;
  // split time in the number of frames i have and check the closest vehicle at that time
  for (int i = 0; i < constants::FRAMES_PER_SEC * tr.duration; i++) {
    double s_at_time = p_s.evalAt(i * dt, 0);
    double d_at_time = p_d.evalAt(i * dt, 0);
    for (auto &o_car : r.cars) {
      auto other = o_car.getStateAt(i * dt);
      auto diff_s = s_at_time - other[0];
      auto diff_d = d_at_time - other[1];
      auto d2 = pow(diff_d, 2) + pow(diff_s, 2);
      if (d2 <= d_closest) {
        d_closest = d2;
      }
    }
  }
  return logistic(constants::SAFETY_DISTANCE / d_closest);
}

double TrajectoryGenerator::maxAccelerationCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r) {
  Polynomial p_s(tr.s_trajectory);
  Polynomial p_d(tr.d_trajectory);
  double dt = 1.0 / constants::FRAMES_PER_SEC;

  for (int i = 0; i < constants::FRAMES_PER_SEC * tr.duration; i++) {
    auto s_a = p_s.evalAt(i * dt, 2);
    auto d_a = p_d.evalAt(i * dt, 2);
    auto acc = sqrt(s_a * s_a + d_a * d_a);
    if (acc > constants::MAX_JERK) {
      return 1.0;
    }
  }
  return 0.0;
}

double TrajectoryGenerator::maxJerkCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r) {
  Polynomial p_s(tr.s_trajectory);
  Polynomial p_d(tr.d_trajectory);
  double dt = 1.0 / constants::FRAMES_PER_SEC;

  for (int i = 0; i < constants::FRAMES_PER_SEC * tr.duration; i++) {
    auto s_j = p_s.evalAt(i * dt, 3);
    auto d_j = p_d.evalAt(i * dt, 3);
    auto j = sqrt(s_j * s_j + d_j * d_j);
    if (j > constants::MAX_JERK) {
      return 1.0;
    }
  }
  return 0.0;
}

double TrajectoryGenerator::maxSpeedCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r) {
  Polynomial p_s(tr.s_trajectory);
  Polynomial p_d(tr.d_trajectory);
  double dt = 1.0 / constants::FRAMES_PER_SEC;

  for (int i = 0; i < constants::FRAMES_PER_SEC * tr.duration; i++) {
    auto s_v = p_s.evalAt(i * dt, 1);
    auto d_v = p_d.evalAt(i * dt, 1);
    auto v = sqrt(s_v * s_v + d_v * d_v);
    if (v > constants::MAX_SPEED) {
      return 1.0;
    }
  }
  return 0.0;
}

double TrajectoryGenerator::efficiencyCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r) {
  Polynomial p_s(tr.s_trajectory);
  auto max_distance = tr.duration * constants::TARGET_SPEED; // going full speed all the way
  auto car_distance = p_s.evalAt(tr.duration, 0) - p_s.evalAt(0.0, 0);
  return abs(logistic((max_distance - car_distance) / max_distance));
}

double TrajectoryGenerator::totalJerkCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r) {
  Polynomial p_s(tr.s_trajectory);
  Polynomial p_d(tr.s_trajectory);
  double cost = 0.0;
  double dt = 1.0 / constants::FRAMES_PER_SEC;

  for (int i = 0; i < tr.duration * constants::TARGET_SPEED; i++) {
    auto s_j = p_s.evalAt(i * dt, 3);
    auto d_j = p_d.evalAt(i * dt, 3);
    auto acc = sqrt(s_j * s_j + d_j * d_j);
    cost += acc;
  }
  if (cost / tr.duration > 0.8 * constants::MAX_JERK) { return 1.0; }
  return 0.0;
}

double TrajectoryGenerator::totalAccelerationCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r) {
  Polynomial p_s(tr.s_trajectory);
  Polynomial p_d(tr.s_trajectory);
  double cost = 0.0;
  double dt = 1.0 / constants::FRAMES_PER_SEC;

  for (int i = 0; i < tr.duration * constants::TARGET_SPEED; i++) {
    auto s_a = p_s.evalAt(i * dt, 2);
    auto d_a = p_d.evalAt(i * dt, 2);
    auto acc = sqrt(s_a * s_a + d_a * d_a);
    cost += acc;
  }
  if (cost / tr.duration > 0.8 * constants::MAX_ACCELERATION) { return 1.0; }
  return 0.0;
}

double TrajectoryGenerator::slowLaneChangeCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r) {
  return 0;
}

double TrajectoryGenerator::totalCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r) {
  double cost = 0;
  cost += constants::WEIGHT_TIME_DIFFERENCE_COST * timeDifferenceCost(tr, s, car, r);
  cost += constants::WEIGHT_S_DIFFERENCE_COST * s_DifferenceCost(tr, s, car, r);
  cost += constants::WEIGHT_D_DIFFERENCE_COST * d_DifferenceCost(tr, s, car, r);
  cost += constants::WEIGHT_COLLISION_COST * collisionCost(tr, s, car, r);
  //cost += constants::WEIGHT_BUFFER_COST * bufferCost(tr, s, car, r);
  cost += constants::WEIGHT_MAX_ACCELERATION_COST * maxAccelerationCost(tr, s, car, r);
  cost += constants::WEIGHT_MAX_JERK_COST * maxJerkCost(tr, s, car, r);
  cost += constants::WEIGHT_MAX_SPEED_COST * maxSpeedCost(tr, s, car, r);
  cost += constants::WEIGHT_EFFICIENCY_COST * efficiencyCost(tr, s, car, r);
  cost += constants::WEIGHT_TOTAL_JERK_COST * totalJerkCost(tr, s, car, r);
  cost += constants::WEIGHT_TOTAL_ACCELERATION_COST * totalAccelerationCost(tr, s, car, r);
  cost += constants::WEIGHT_EFFICIENCY_COST * slowLaneChangeCost(tr, s, car, r);

  return cost;
}
