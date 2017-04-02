#include <iostream>
#include "ukf.h"

/**
* Initializes Unscented Kalman filter
*/
UKF::UKF() {
	is_initialized_ = false;

	use_laser_ = true; // if this is false, laser measurements will be ignored (except during init)
	use_radar_ = true; // if this is false, radar measurements will be ignored (except during init)

	std_a_ = 3.0;			// Process noise standard deviation longitudinal acceleration in m/s^2
	std_yawdd_ = M_PI/ 2;	// Process noise standard deviation yaw acceleration in rad/s^2
	std_laspx_ = 0.15;	// Laser measurement noise standard deviation position1 in m
	std_laspy_ = 0.15;	// Laser measurement noise standard deviation position2 in m 
	std_radr_ = 0.3;	// Radar measurement noise standard deviation radius in m
	std_radphi_ = 0.03;	// Radar measurement noise standard deviation angle in rad
	std_radrd_ = 0.3;	// Radar measurement noise standard deviation radius change in m/s
	std_inf_ = 1.0;

	NIS_laser_ = 0.0;
	NIS_radar_ = 0.0;

	InitSensorMatrices();
	InitStateMatrices();
}

UKF::~UKF() {}


// Initialization Code
void UKF::InitSensorMatrices()
{
	n_x_ = 5;	///* State dimension
	n_aug_ = 7;	///* Augmented State dimension


	H_laser_ = MatrixXd(2, n_x_);
	H_laser_ <<
		1, 0, 0, 0, 0,
		0, 1, 0, 0, 0;

	R_radar_ = MatrixXd::Zero(3, 3);
	R_radar_(0, 0) = pow(std_radr_, 2);
	R_radar_(1, 1) = pow(std_radphi_, 2);
	R_radar_(2, 2) = pow(std_radrd_, 2);

	R_laser_ = MatrixXd::Zero(2, 2);
	R_laser_(0, 0) = pow(std_laspx_, 2);
	R_laser_(1, 1) = pow(std_laspy_, 2);
}

void UKF::InitStateMatrices()
{
	n_x_ = 5;
	n_aug_ = 7;
	int n_noise = n_aug_ - n_x_;

	lambda_ = 3 - n_aug_;
	int radar_dim = 3;


	x_ = VectorXd::Zero(n_x_);;	 // initial state vector
	xn_ = VectorXd(n_noise); // initial noise vector
	P_ = MatrixXd(n_x_, n_x_);	// initial covariance matrix
	Q_ = MatrixXd(n_noise, n_noise);

	z_prediction_ = VectorXd(radar_dim);
	S_ = MatrixXd(3, radar_dim);
	T_ = MatrixXd(n_x_, radar_dim);

	P_ <<
		std_inf_, 0, 0, 0, 0,
		0, std_inf_, 0, 0, 0,
		0, 0, std_inf_, 0, 0,
		0, 0, 0, std_inf_, 0,
		0, 0, 0, 0, std_inf_;

	Q_ <<
		std_a_ * std_a_, 0,
		0, std_yawdd_ * std_yawdd_;

	I_ = MatrixXd::Identity(n_x_, n_x_);
	weights_ = GenerateSigmaWeights(n_aug_, lambda_);
}

void UKF::Initialize(MeasurementPackage& m)
{
	double px = 0;
	double py = 0;

	if (m.sensor_type_ == MeasurementPackage::RADAR) {
		double rho = m.raw_measurements_[0];
		double phi = m.raw_measurements_[1];
		double rho_dot = m.raw_measurements_[2];

		px = rho * cos(phi);
		py = rho * sin(phi);
		x_ << px, py, rho_dot, 0.0, 0.0;
	}
	else if (m.sensor_type_ == MeasurementPackage::LASER) {
		px = m.raw_measurements_[0];
		py = m.raw_measurements_[1];
		x_ << px, py, 0.0, 0.0, 0.0;
	}


	xn_.fill(0); // initial noise vector is zero

	previous_timestamp_ = m.timestamp_;

	is_initialized_ = true;
}

// Sigma Points Creation and Mapping
VectorXd UKF::GenerateSigmaWeights(int stateDimension, int lamdaParameter)
{
	VectorXd weights = VectorXd(2 * stateDimension + 1);

	weights.segment(1, 2 * stateDimension).fill(0.5 / (stateDimension + lamdaParameter));
	weights(0) = (double)lamdaParameter / (lamdaParameter + stateDimension);

	return weights;
}

void UKF::GenerateSigmaPoints(int stateDimension, int lamdaParameter)
{
	sigmaPoints_ = MatrixXd(stateDimension, 2 * stateDimension + 1);

	VectorXd x_aug = VectorXd(stateDimension);
	MatrixXd P_aug = MatrixXd::Zero(stateDimension, stateDimension);

	x_aug << x_.array(), xn_.array();

	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug.bottomRightCorner(Q_.rows(), Q_.cols()) = Q_;

	// std::cout << P_aug;

	MatrixXd A = P_aug.llt().matrixL();
	MatrixXd offset = A * sqrt(lamdaParameter + stateDimension);

	sigmaPoints_.colwise() = x_aug;
	sigmaPoints_.block(0, 1, stateDimension, stateDimension) += offset;
	sigmaPoints_.block(0, stateDimension + 1, stateDimension, stateDimension) -= offset;
	// std::cout << sigmaPoints_;

	return;
}

void UKF::PredictStateOfSigmaPoint(int sigmaPoint, double dt)
{
	double p_x = sigmaPoints_(0, sigmaPoint);
	double p_y = sigmaPoints_(1, sigmaPoint);
	double v = sigmaPoints_(2, sigmaPoint);
	double yaw = sigmaPoints_(3, sigmaPoint);
	double yawd = sigmaPoints_(4, sigmaPoint);
	double nu_a = sigmaPoints_(5, sigmaPoint);
	double nu_yawdd = sigmaPoints_(6, sigmaPoint);

	// predicted values
	double px_p, py_p, v_p, yaw_p, yawd_p;

	if (fabs(yawd) > 0.001) {
		px_p = p_x + v / yawd * (sin(yaw + yawd * dt) - sin(yaw));
		py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * dt));
	}
	else {
		px_p = p_x + v * dt * cos(yaw);
		py_p = p_y + v * dt * sin(yaw);
	}

	v_p = v;
	yaw_p = yaw + yawd*dt;
	yawd_p = yawd;

	//add noise
	px_p = px_p + 0.5 * nu_a * dt * dt * cos(yaw);
	py_p = py_p + 0.5 * nu_a * dt * dt * sin(yaw);
	v_p = v_p + nu_a * dt;

	yaw_p = yaw_p + 0.5 * nu_yawdd * dt * dt;
	yawd_p = yawd_p + nu_yawdd * dt;

	sigmaPoints_.col(sigmaPoint) << px_p, py_p, v_p, yaw_p, yawd_p, 0, 0;

	return;
}

void UKF::PredictMeasurementForSigmaPoint(int sigmaPoint)
{
	double p_x = sigmaPoints_(0, sigmaPoint);
	double p_y = sigmaPoints_(1, sigmaPoint);
	double v = sigmaPoints_(2, sigmaPoint);
	double yaw = sigmaPoints_(3, sigmaPoint);

	double v_y = cos(yaw) * v;
	double v_x = sin(yaw) * v;

	// measurement model
	double rho = sqrt(p_x * p_x + p_y * p_y);
	double phi = atan2(p_y, p_x);
	double rho_dot = (p_x * v_y + p_y * v_x) / rho;

	if (rho != rho) {
		rho = 0;
	}
	if (phi != phi) {
		phi = 0;
	}
	if (rho_dot != rho_dot) {
		rho_dot = 0;
	}


	ZsigmaPoints_.col(sigmaPoint) << rho, phi, rho_dot;

	return;
}

// UKF Methods
void UKF::ProcessMeasurement(MeasurementPackage m) {
	if (!is_initialized_) {
		// first measurement
		Initialize(m);
		return;
	}

	Prediction(m);
	Update(m);

}

// predict
void UKF::Prediction(MeasurementPackage m) {
	double dt = (m.timestamp_ - previous_timestamp_) / 1e6;

	// large time intervals cause instability
	while (dt > 0.1) {
		GenerateSigmaPoints(n_aug_, lambda_);
		PredictNextState(0.05);
		dt -= 0.05;
	}

	previous_timestamp_ = m.timestamp_;

	GenerateSigmaPoints(n_aug_, lambda_);
	PredictNextState(dt);

}

void UKF::PredictNextState(double dt)
{
	x_.fill(0.0);
	P_.fill(0.0);

	VectorXd offset;

	//iterate over sigma points
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		PredictStateOfSigmaPoint(i, dt);
		// calculate state mean
		x_ = x_ + weights_(i) * sigmaPoints_.col(i).topRows(n_x_);
		x_(3) = NormalizeAngle(x_(3));

	}

	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		// calculate covariance matrix
		offset = sigmaPoints_.col(i).topRows(n_x_) - x_;
		offset(3) = NormalizeAngle(offset(3));
		P_ = P_ + weights_(i) * offset * offset.transpose();
	}

	return;
}

void UKF::PredictNextRadarMeasurement()
{
	VectorXd z_offset, x_offset;

	ZsigmaPoints_ = MatrixXd(3, 2 * n_aug_ + 1);

	z_prediction_.fill(0.0);
	ZsigmaPoints_.fill(0.0);
	S_.fill(0.0);
	T_.fill(0.0);

	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		PredictMeasurementForSigmaPoint(i);
		z_prediction_ = z_prediction_ + weights_(i) * ZsigmaPoints_.col(i);
	}

	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		z_offset = ZsigmaPoints_.col(i) - z_prediction_;
		x_offset = sigmaPoints_.col(i).topRows(n_x_) - x_;

		z_offset(1) = NormalizeAngle(z_offset(1));
		x_offset(1) = NormalizeAngle(x_offset(1));

		S_ = S_ + weights_(i) * z_offset * z_offset.transpose();
		T_ = T_ + weights_(i) * x_offset * z_offset.transpose();
	}

	S_ = S_ + R_radar_;
}

// update
void UKF::Update(MeasurementPackage m) {
	if (m.sensor_type_ == MeasurementPackage::RADAR) {
		UpdateRadar(m);
	}
	else {
		UpdateLidar(m);
	}
}

void UKF::UpdateLidar(MeasurementPackage m) {
	VectorXd z = m.raw_measurements_;

	VectorXd z_pred = H_laser_ * x_;
	VectorXd z_diff = z - z_pred;
	MatrixXd Ht = H_laser_.transpose();
	MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd K = P_ * Ht * Si;

	//new estimate
	x_ = x_ + (K * z_diff);
	P_ = (I_ - K * H_laser_) * P_;

	NIS_laser_ = z_diff.transpose() * Si * z_diff;
}

void UKF::UpdateRadar(MeasurementPackage m) {
	// calculating T, S and z_prediction from sigma points
	PredictNextRadarMeasurement();

	MatrixXd K = T_ * S_.inverse();

	VectorXd z = m.raw_measurements_;
	VectorXd residual = z - z_prediction_;

	residual(1) = NormalizeAngle(residual(1));

	//update state mean and covariance matrix
	x_ = x_ + K * residual;
	x_(3) = NormalizeAngle(x_(3));
	P_ = P_ - K * S_ * K.transpose();

	NIS_radar_ = residual.transpose() * S_.inverse() * residual;
}

// extra functions
double UKF::NormalizeAngle(double angle)
{
	const double Max = M_PI;
	const double Min = -M_PI;

	return angle < Min
		? Max + std::fmod(angle - Min, Max - Min)
		: std::fmod(angle - Min, Max - Min) + Min;
}

VectorXd UKF::CalculateRMSE(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truth)
{
	// assert estimations and ground truth vectors are of equal size and not empty
	assert(estimations.size() > 0);
	assert(estimations.size() == ground_truth.size());

	VectorXd rmse(4);
	rmse << 0.0, 0.0, 0.0, 0.0; // initialize root mean square error 

	for (size_t i = 0; i < estimations.size(); ++i)
	{
		VectorXd error = estimations[i] - ground_truth[i];
		error = error.array() * error.array();
		rmse += error;
	}

	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();
	return rmse;
}
