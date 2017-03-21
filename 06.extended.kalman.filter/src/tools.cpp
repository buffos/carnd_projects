#include <iostream>
#include "tools.h"

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truth) {
	// assert estimations and ground truth vectors are of equal size and not empty
	assert(estimations.size() > 0);
	assert(estimations.size() == ground_truth.size());

	VectorXd rmse(4);
	rmse << 0.0, 0.0, 0.0, 0.0; // initialize root mean square error 

	for (int i = 0; i < estimations.size(); ++i)
	{
		VectorXd error = estimations[i] - ground_truth[i];
		error = error.array() * error.array();
		rmse += error;
	}

	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();
	return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3, 4);

	const double px = x_state(0);
	const double py = x_state(1);
	const double vx = x_state(2);
	const double vy = x_state(3);

	const double c1 = std::max(1.0e-6, px*px + py*py);
	const double c2 = sqrt(c1);
	const double c3 = c1*c2;

	// the Jacobian matrix
	Hj << px / c2, py / c2, 0, 0,
		-py / c1, px / c1, 0, 0,
		py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;

	return Hj;
}


VectorXd Tools::PolarToCartesian(double r, double phi) {
	VectorXd toCartesian = VectorXd(2);
	toCartesian << r * cos(phi), r * sin(phi);
	return toCartesian;
}
