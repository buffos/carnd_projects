#include "PID.h"
#include <iostream>
#include <numeric>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(string name) : str_name_(name), p_error_(0.0), i_error_(0.0), d_error_(0.0),
twiddle_iterations_(0), twiddle_mode_(0),twiddle_optimizingIndex_(0), calibration_on_(false) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;

	p_error_ = 0.0;
	i_error_ = 0.0;
	d_error_ = 0.0;
}

void PID::UpdateError(double cte) {
	double previous_cte = p_error_;

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

	calibration_on_ = true;

	twiddle_optimizingIndex_ = 0;
	twiddle_iterations_ = 0;
	twiddle_mode_ = 0;

	twiddle_bestError_ = 1000000.;
}

void PID::calibrationRESET() {
	twiddle_optimizingIndex_ = 0;
	twiddle_mode_ = 0;

	twiddle_bestError_ = 1000000.;

	Init(p_[0], p_[1], p_[2]);
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

	if (twiddle_mode_ == 3 || calibration_on_ == false) { // threshold reached or calibration is not active
		calibrationOFF();
		cout << str_name_ << "*************OFF*************" << endl;
		return; // no action required
	}

	cout << str_name_ << "  PID Error: " << TotalError() << ", Best Error: " << twiddle_bestError_ << ", iteration: " << twiddle_iterations_ << endl;

	if (twiddle_iterations_ < 1000) {
		twiddle_iterations_++;
		return;
	}

	if (twiddle_mode_ == 0) { // beggining of the loop
		p_[twiddle_optimizingIndex_] += dp_[twiddle_optimizingIndex_];
		UpdateCoefficients(p_[0], p_[1], p_[2]); // update coefficients
		twiddle_mode_ = 1; // mode == 1 means increasing dp_
		return;
	}

	if (twiddle_mode_ == 1) { // increase mode
		if (std::abs(TotalError()) < twiddle_bestError_) {
			twiddle_bestError_ = std::abs(TotalError());
			dp_[twiddle_optimizingIndex_] *= 1.1;
		}
		else {
			p_[twiddle_optimizingIndex_] -= 2 * dp_[twiddle_optimizingIndex_]; // p+dp-2dp = p-dp
			UpdateCoefficients(p_[0], p_[1], p_[2]); // update coefficients
			twiddle_mode_ = 2; // mode == 2 means decreasing dp_
			return;
		}
	}
	else if (twiddle_mode_ == 2) { // twiddle_mode_ == 2
		if (std::abs(TotalError()) < twiddle_bestError_) {
			twiddle_bestError_ = std::abs(TotalError());
			dp_[twiddle_optimizingIndex_] *= 1.1;
		}
		else {
			p_[twiddle_optimizingIndex_] += dp_[twiddle_optimizingIndex_]; // p+dp-2dp+dp = p
			UpdateCoefficients(p_[0], p_[1], p_[2]); // update coefficients
			dp_[twiddle_optimizingIndex_] *= 0.9;
		}
	}

	double sum = std::accumulate(dp_.begin(), dp_.end(), 0.);

	if (sum < PID::dp_tolerance_ && twiddle_iterations_ > PID::max_iterations_) {
		twiddle_mode_ = 3; // signals stop twiddling 
	}
	else {
		twiddle_mode_ = 0; // signals next loop
		twiddle_optimizingIndex_ = (twiddle_optimizingIndex_ + 1) % 3;
		twiddle_iterations_++;
	}

}
