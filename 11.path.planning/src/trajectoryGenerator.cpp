#include "trajectoryGenerator.h"
#include "polynomial.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/// helper functions

double logistic(double x)
{
    // A function that returns a value between 0 and 1 for x in the range [0, infinity]
    //  and -1 to 1 for x in the range [-infinity, infinity].
    return 2.0 / (1 + exp(-x)) - 1.0;
}

Trajectory TrajectoryGenerator::generateTrajectory(StateGoal &s, Vehicle &car, Road &r)
{
    double lowTimeBound = planDuration - 4 * timestep;
    double highTimeBound = planDuration + 4 * timestep;

    vector<StateGoal> newGoals = std::move(perturbGoal(s));
    vector<Trajectory> newTrajectories;

    // for all plan duration times and all perturbations
    // of the original goal I create all possible trajectories
    for (double time = lowTimeBound; time <= highTimeBound; time += timestep)
    {
        for (auto goal : newGoals)
        {
            Trajectory tr;
            tr.s_trajectory = std::move(jmt(goal, time, 1));
            tr.d_trajectory = std::move(jmt(goal, time, 2));
            tr.duration = time;
            tr.cost = totalCost(tr, s, car, r); // find cost of trajectory
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
vector<StateGoal> TrajectoryGenerator::perturbGoal(StateGoal &s)
{
    vector<StateGoal> newGoals;
    std::default_random_engine generator; // to generate the random samples

    std::normal_distribution<double> distribution_s_position(s.end_s[0], s.end_s[0] * sigma);
    std::normal_distribution<double> distribution_s_velocity(s.end_s[1], s.end_s[1] * sigma);
    std::normal_distribution<double> distribution_s_acceleration(s.end_s[2], s.end_s[2] * sigma);
    std::normal_distribution<double> distribution_d_position(s.end_d[0], s.end_d[0] * sigma);
    std::normal_distribution<double> distribution_d_velocity(s.end_d[1], s.end_d[1] * sigma);
    std::normal_distribution<double> distribution_d_acceleration(s.end_d[2], s.end_d[2] * sigma);

    for (int i = 0; i < numberOfSamples; i++)
    {
        StateGoal newStateGoal;
        newStateGoal.start_s = s.start_s;
        newStateGoal.start_d = s.start_d;
        newStateGoal.end_s = {distribution_s_position(generator),
                              distribution_s_velocity(generator),
                              distribution_s_acceleration(generator)};
        newStateGoal.end_d = {distribution_d_position(generator),
                              distribution_d_velocity(generator),
                              distribution_d_acceleration(generator)};
        newGoals.push_back(newStateGoal);
    }
    return newGoals;
}

vector<double> TrajectoryGenerator::jmt(StateGoal &s, double t, int s_or_d)
{
    MatrixXd A = MatrixXd(3, 3);
    VectorXd B = MatrixXd(3, 1);

    vector<double> start;
    vector<double> end;
    if (s_or_d == 1)
    {
        start = s.start_s;
        end = s.end_s;
    }
    else
    {
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

// cost because the trajectory is shorter or longer than the desired planDuration
double TrajectoryGenerator::timeDifferenceCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r)
{
    double x = abs(planDuration - tr.duration) / planDuration;
    return logistic(x);
}

// cost because the s coordinate and derivatives are not as the goal
double TrajectoryGenerator::s_DifferenceCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r)
{
    Polynomial p(tr.s_trajectory); // create a polynomial based on the trajectory coefficients
    double actual_s_s = p.evalAt(planDuration, 0);
    double actual_s_v = p.evalAt(planDuration, 1);
    double actual_s_a = p.evalAt(planDuration, 2);

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

// cost because the d coordinate and derivatives are not as the goal
double TrajectoryGenerator::d_DifferenceCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r)
{
    Polynomial p(tr.s_trajectory); // create a polynomial based on the trajectory coefficients
    double actual_d_s = p.evalAt(planDuration, 0);
    double actual_d_v = p.evalAt(planDuration, 1);
    double actual_d_a = p.evalAt(planDuration, 2);

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
double TrajectoryGenerator::collisionCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r)
{
    Polynomial p_s(tr.s_trajectory);
    Polynomial p_d(tr.d_trajectory);

    double closest = 9999999.0; // at any time
    double dt = planDuration / 100.;
    // split time in the number of frames i have and check the closest vehicle at that time
    for (double t = 0; t <= planDuration; t += dt)
    {
        double s_at_time = p_s.evalAt(t, 0);
        double d_at_time = p_d.evalAt(t, 0);
        double closest_at_time = r.closestVehicleAt(s_at_time, d_at_time, t);
        if (closest_at_time < closest)
        {
            closest = closest_at_time;
        }
    }
    double cost = (closest < 2 * car.safetyRadius) ? 1.0 : 0.0;
    return cost;
}

double TrajectoryGenerator::bufferCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r)
{
    Polynomial p_s(tr.s_trajectory);
    Polynomial p_d(tr.d_trajectory);

    double closest = 9999999.0; // at any time
    double dt = planDuration / 100.;
    // split time in the number of frames i have and check the closest vehicle at that time
    for (double t = 0; t <= planDuration; t += dt)
    {
        double s_at_time = p_s.evalAt(t, 0);
        double d_at_time = p_d.evalAt(t, 0);
        double closest_at_time = r.closestVehicleAt(s_at_time, d_at_time, t);
        if (closest_at_time < closest)
        {
            closest = closest_at_time;
        }
    }
    double cost = logistic(2 * car.safetyRadius / closest);
    return cost;
}

double TrajectoryGenerator::maxAccelerationCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r)
{
    Polynomial p_s(tr.s_trajectory);
    double dt = planDuration / 100.;
    double max_acceleration = 0.0;
    for (double t = 0; t <= planDuration; t += dt)
    {
        double s_acc_at_time = p_s.evalAt(t, 2);
        max_acceleration = (s_acc_at_time > max_acceleration) ? s_acc_at_time : max_acceleration;
    }
    double cost = logistic(max_acceleration / r.max_acceleration);
    return cost;
}

double TrajectoryGenerator::maxJerkCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r)
{
    Polynomial p_s(tr.s_trajectory);
    double dt = planDuration / 100.;
    double max_jerk = 0.0;
    for (double t = 0; t <= planDuration; t += dt)
    {
        double s_jerk_at_time = p_s.evalAt(t, 3);
        max_jerk = (s_jerk_at_time > max_jerk) ? s_jerk_at_time : max_jerk;
    }
    double cost = logistic(max_jerk / r.max_jerk);
    return cost;
}

double TrajectoryGenerator::totalCost(Trajectory &tr, StateGoal &s, Vehicle &car, Road &r)
{
    double cost = 0;

    cost += 1.0 * timeDifferenceCost(tr, s, car, r);
    cost += 100 * s_DifferenceCost(tr, s, car, r);
    cost += 10000 * d_DifferenceCost(tr, s, car, r);
    cost += 1000000 * collisionCost(tr, s, car, r);
    cost += 100 * bufferCost(tr, s, car, r);
    cost += 5 * maxAccelerationCost(tr, s, car, r);
    cost += 5 * maxJerkCost(tr, s, car, r);

    return cost;
}