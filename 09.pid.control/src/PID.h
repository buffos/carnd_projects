#ifndef PID_H
#define PID_H

#include <vector>
#include <string>

using namespace std;

class PID {
public:
	/*
	* Static variables
	*/
	static const long max_iterations_ = 1000000;
	static const long min_iterations_ = 100;
	static constexpr double dp_tolerance_ = 0.01;
public:
	string str_name_;
	/*
	* twiddling variables
	*/
	bool calibration_on_;

	int twiddle_optimizingIndex_;
	long twiddle_iterations_;
	int twiddle_mode_;

	double twiddle_bestError_;

	vector<double> p_;
	vector<double> dp_;

	/*
	* Errors
	*/
	double p_error_;
	double i_error_;
	double d_error_;

	/*
	* Coefficients
	*/
	double Kp_;
	double Ki_;
	double Kd_;

	/*
	* Constructor
	*/
	PID(string name);

	/*
	* Destructor.
	*/
	virtual ~PID();

	/*
	* Initialize PID.
	*/
	void Init(double Kp, double Ki, double Kd);

	/*
	* Update Coeeficients.
	*/
	void UpdateCoefficients(double Kp, double Ki, double Kd);

	/*
	* Update the PID error variables given cross track error.
	*/
	void UpdateError(double cte);

	/*
	* Calculate the total PID error.
	*/
	double TotalError();

	/*
	* Calibration functions.
	*/
	void calibrationON(vector<double> p, vector<double> dp);
	void calibrationOFF() { calibration_on_ = false; };
	void calibrationRESET();

	void twiddle();
};

#endif /* PID_H */
