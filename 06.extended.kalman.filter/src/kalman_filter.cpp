#include "kalman_filter.h"

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}


void KalmanFilter::Init( VectorXd& x_in,  MatrixXd& P_in,  MatrixXd& F_in,  MatrixXd& Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
  I_ = MatrixXd::Identity(P_.rows(), P_.cols());
}

void KalmanFilter::Predict()
{
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd& residual, const MatrixXd& H, const MatrixXd& R)
{
	const MatrixXd Ht = H.transpose();
	const MatrixXd S = H * P_ * Ht + R; // innovation (or residual) covariance
	const MatrixXd K = P_ * Ht * S.inverse(); // Kalman Gain

	//new estimate
	x_ = x_ + (K * residual); // posterior state estimate
	P_ = (I_ - K * H) * P_;  // posterior state covariance
}
