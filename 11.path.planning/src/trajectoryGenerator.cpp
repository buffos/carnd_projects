#include "trajectoryGenerator.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
            tr.evaluation = 0.0;
            newTrajectories.push_back(tr);
        }
    } 

    // now we evaluate the generated trajectories and select the best trajectory
    return Trajectory();
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