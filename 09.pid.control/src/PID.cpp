#include "PID.h"
#include <iostream>
#include <numeric>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(string name) : str_name_(name), p_error_(0.0), i_error_(0.0), d_error_(0.0), integration_iterations_(0),
twiddle_iterations_(0), twiddle_mode_(0), twiddle_optimizingIndex_(0), twiddle_error_(0.0), twiddler_yield_(false), calibration_on_(false) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;

	p_error_ = 0.0;
	i_error_ = 0.0;
	d_error_ = 0.0;

	integration_iterations_ = 0;
}

void PID::UpdateError(double cte) {
	double previous_cte = p_error_;

	integration_iterations_++;
	twiddler_yield_ = false;

	if (integration_iterations_ > activate_twiddler_every_n_iterations_ - 1) {
		twiddler_yield_ = true; // run twiddler just before reseting the counter
	}

	if (integration_iterations_ > activate_twiddler_every_n_iterations_) {
		integration_iterations_ = 0;
	}

	p_error_ = cte; // Proportional error
	d_error_ = (cte - previous_cte);  // Differential error
	i_error_ += cte;  // Integration error
}

void PID::UpdateCoefficients(double Kp, double Ki, double Kd) {
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;

	std::cout << str_name_ << ": *new* p vector: " << Kp_ << "\t" << Ki_ << "\t" << Kd_ << std::endl;
	std::cout << str_name_ << ": *new* dp vector: " << dp_[0] << "\t" << dp_[1] << "\t" << dp_[2] << std::endl;
}

double PID::TotalError() {
	return -Kp_ * p_error_ - Ki_ * i_error_ - Kd_ * d_error_;
}

void PID::calibrationON(vector<double> p, vector<double> dp) {
	p_ = p;
	dp_ = dp;
	best_p_ = p_;

	calibration_on_ = true;

	twiddle_optimizingIndex_ = 0;
	twiddle_iterations_ = 0;
	twiddle_mode_ = 0;
	twiddle_error_ = 0.0;

	twiddle_bestError_ = 1000000.;
}

void PID::calibrationRESET() {
	i_error_ = 0;
	p_error_ = 0;
	d_error_ = 0;
	twiddle_mode_ = 0;

	integration_iterations_ = 0;
	twiddler_yield_ = false;
	twiddle_optimizingIndex_ = (twiddle_optimizingIndex_ + 1) % 3;

	if (calibration_on_) {
		p_ = best_p_; // restore p_ for last best_error
		dp_[twiddle_optimizingIndex_] *= 0.9;

		std::cout << " Restoring p_ after crash: " << p_[0] << "\t" << p_[1] << "\t" << p_[2] << std::endl;
		std::cout << " Restoring dp_ after crash: " << dp_[0] << "\t" << dp_[1] << "\t" << dp_[2] << std::endl;
		Init(p_[0], p_[1], p_[2]);
	}

}

void PID::twiddle() {
	// every time it runs it executes one optimizing loop like the python algorithm below

	// -----------  python twiddle loop (trun's pid tutorial) ----------------
	// while sum(dp) > tol:
	//   for i in range(len(p)):   --> mode 0
	//     p[i] += dp[i]
	//     _, _, err = run(robot, p) --> mode 1
	//     if err < best_err:
	//         best_err = err
	//         dp[i] *= 1.1
	//     else:
	//         p[i] -= 2.0 * dp[i]
	//         _, _, err = run(robot, p) --> mode 2
	//         if err < best_err:
	//             best_err = err
	//             dp[i] *= 1.1
	//        else:
	//             p[i] += dp[i]
	//             dp[i] *= 0.9
	// --------------------------------------------------

	if (calibration_on_ == false) { // threshold reached or calibration is not active
		return; // no action required
	}

	if (integration_iterations_ < activate_twiddler_every_n_iterations_ / 2) {
		twiddle_error_ = 0.0; // accumulate error only in the second half of the iterations.
		return; // the first half iterations are skipped
	}
	else {
		twiddle_error_ += p_error_ * p_error_; // cte*cte. accumulate error
	}

	if (twiddler_yield_ == false) {
		return; // not time to run twiddler yet
	}

	// twiddling......
	std::cout << "Twiddling: " << "best error: " << twiddle_bestError_ << "best p_: " << best_p_[0] << "\t" << best_p_[1] << "\t" << best_p_[2] << std::endl;

	if (twiddle_error_ < twiddle_bestError_ && twiddle_iterations_>0) {
		best_p_ = p_; // save in case of crash the best p_ vector
		twiddle_bestError_ = twiddle_error_; // 
		dp_[twiddle_optimizingIndex_] *= 1.1;
	}
	else if (twiddle_mode_ == 0) { // beggining of the loop
		p_[twiddle_optimizingIndex_] += dp_[twiddle_optimizingIndex_];
		std::cout << "Iteration: " << twiddle_iterations_ << " index: " << twiddle_optimizingIndex_ << " Trying increased p_: " << p_[0] << "\t" << p_[1] << "\t" << p_[2] << std::endl;
		UpdateCoefficients(p_[0], p_[1], p_[2]); // update coefficients
		twiddle_mode_ = 1; // mode == 1 means increasing dp_
		i_error_ = 0; // before beginning a new optimization loop reset integration error
		return;
	}
	else if (twiddle_mode_ == 1) { // increase mode
		p_[twiddle_optimizingIndex_] -= 2 * dp_[twiddle_optimizingIndex_]; // p+dp-2dp = p-dp
		std::cout << "Iteration: " << twiddle_iterations_ << " index: " << twiddle_optimizingIndex_ << " Trying decreased p_: " << p_[0] << "\t" << p_[1] << "\t" << p_[2] << std::endl;
		UpdateCoefficients(p_[0], p_[1], p_[2]); // update coefficients
		twiddle_mode_ = 2; // mode == 2 means decreasing dp_
		i_error_ = 0; // before beginning a new optimization loop reset integration error
		return;
	}
	else if (twiddle_mode_ == 2) { // twiddle_mode_ == 2
		// reverting to  previous p value since increase and decreasing did not succeed
		p_[twiddle_optimizingIndex_] += dp_[twiddle_optimizingIndex_]; // p+dp-2dp+dp = p
		std::cout << "Iteration: " << twiddle_iterations_ << " index: " << twiddle_optimizingIndex_ << "Restoring p_: " << p_[0] << "\t" << p_[1] << "\t" << p_[2] << std::endl;
		UpdateCoefficients(p_[0], p_[1], p_[2]); // update coefficients
		dp_[twiddle_optimizingIndex_] *= 0.9; // if i reach here the optimization for the index failed
	}

	double sum = std::accumulate(dp_.begin(), dp_.end(), 0.);

	if (sum < PID::dp_tolerance_ && twiddle_iterations_ > PID::max_iterations_) {
		calibration_on_ = false; // signals stop twiddling 
	}
	else {
		twiddle_mode_ = 0; // signals next loop
		twiddle_optimizingIndex_ = (twiddle_optimizingIndex_ + 1) % 3;
		twiddle_iterations_++;
		i_error_ = 0;
	}

}
