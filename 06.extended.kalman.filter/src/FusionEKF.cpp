#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


FusionEKF::FusionEKF()
{
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);


	const double var_px = 0.0225;
	const double var_py = 0.0225;
	const double var_rho = 0.09;
	const double var_phi = 0.0009;
	const double var_rho_dot = 0.09;
	const double var_inf = 10000.0;

	H_laser_ << 
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0;

	R_laser_ << 
		var_px, 0.0,
		0.0, var_py;


	R_radar_ << 
		var_rho, 0.0, 0.0,
		0.0, var_phi, 0.0,
		0.0, 0.0, var_rho_dot;

	ekf_.x_ = VectorXd(4);

	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, var_inf, 0.0,
		0.0, 0.0, 0.0, var_inf;

	ekf_.I_ = MatrixXd::Identity(ekf_.x_.size(), ekf_.x_.size());

	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << 
		1, 0, 1, 0,
		0, 1, 0, 1,
		1, 0, 1, 0,
		0, 1, 0, 1;

	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ <<
		1.0, 0.0, 1.0, 0.0,
		0.0, 1.0, 0.0, 1.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0;
}


FusionEKF::~FusionEKF() {}


void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack)
{
	if (!is_initialized_)
	{
		is_initialized_ = true;
		Initialize(measurement_pack);
		return;
	}

	Predict(measurement_pack);
	Update(measurement_pack);
}


void FusionEKF::Initialize(const MeasurementPackage& m)
{
	// first measurement
	previous_timestamp_ = m.timestamp_;

	if (m.sensor_type_ == MeasurementPackage::RADAR)
	{
		const double r = m.raw_measurements_[0];
		const double phi = m.raw_measurements_[1];
		const double r_dot = m.raw_measurements_[2];

		VectorXd position = tools.PolarToCartesian(r, phi);
		VectorXd velocity = tools.PolarToCartesian(r_dot, phi);
		ekf_.x_ << position[0], position[1], velocity[0], velocity[1];
	}
	else if (m.sensor_type_ == MeasurementPackage::LASER)
	{
		ekf_.x_ << m.raw_measurements_[0], m.raw_measurements_[1], 0.0, 0.0;
	}
}


void FusionEKF::Predict(const MeasurementPackage& m)
{
	const long new_timestamp = m.timestamp_;
	const double dt = (new_timestamp - previous_timestamp_) / 1.0e6;

	if (dt < 0.001)
	{
		// for simultaneous measurements use current prediction
		return;
	}

	previous_timestamp_ = new_timestamp;

	Update_Q_Matrix(dt); // update process covariance matrix which depends on the time step
	Update_F_Matrix(dt); // update state covariance matrix which depends on the time step

	ekf_.Predict();
}

VectorXd FusionEKF::PredictRadarMeasurement(const VectorXd& x) const
{
	const double px = x[0];
	const double py = x[1];
	const double vx = x[2];
	const double vy = x[3];
	const double epsilon = 1e-5;
	const double r = sqrt(px * px + py * py);
	const double phi = atan2(py, px);
	const double r_dot = (px * vx + py * vy) / (epsilon + r);

	VectorXd prediction(3);
	prediction << r, phi, r_dot;
	return prediction;
}



void FusionEKF::Update(const MeasurementPackage& m)
{
	if (m.sensor_type_ == MeasurementPackage::RADAR)
	{
		const VectorXd& z = m.raw_measurements_;
		const VectorXd z_prediction = PredictRadarMeasurement(ekf_.x_);
		const VectorXd residual = z - z_prediction;

		Hj_ = tools.CalculateJacobian(ekf_.x_);
		ekf_.Update(residual, Hj_, R_radar_);
	}
	else
	{
		const VectorXd &z = m.raw_measurements_;
		const VectorXd z_prediction = H_laser_ * ekf_.x_;
		const VectorXd residual = z - z_prediction;

		ekf_.Update(residual, H_laser_, R_laser_);
	}
}


void FusionEKF::Update_Q_Matrix(double dt)
{
	const double dt_2 = dt * dt;
	const double dt_3 = dt_2 * dt;
	const double dt_4 = dt_3 * dt;
	const double noise_ax = 9;
	const double noise_ay = 9;

	ekf_.Q_ << 
		dt_4 / 4 * noise_ax,	0,						dt_3 / 2 * noise_ax,	0,
		0,						dt_4 / 4 * noise_ay,	0,						dt_3 / 2 * noise_ay,
		dt_3 / 2 * noise_ax,	0,						dt_2*noise_ax,			0,
		0,						dt_3 / 2 * noise_ay,	0,						dt_2*noise_ay;
}


void FusionEKF::Update_F_Matrix(double dt)
{
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;
}
