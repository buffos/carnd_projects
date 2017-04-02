#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class UKF {
public:


	bool is_initialized_;	///* initially set to false, set to true in first call of ProcessMeasurement
	bool use_laser_;		///* if this is false, laser measurements will be ignored (except for init)
	bool use_radar_;		///* if this is false, radar measurements will be ignored (except for init)


	VectorXd x_;			///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
	VectorXd xn_;			///* noise vector: [noise_yaw_angle noise_yaw_rate] in SI units and rad
	VectorXd z_prediction_; ///* predicted measurement
	MatrixXd P_;			///* state covariance matrix
	MatrixXd Q_;			///* process noise covariance matrix
	MatrixXd I_;			///* Identity Matrix
	MatrixXd S_;			///* measurement covariance matrix E(z,z.t)
	MatrixXd T_;			///* crosscorellation matrix E(x,z.t)

	int n_x_;				///* State dimension
	int n_aug_;				///* Augmented state dimension

	MatrixXd sigmaPoints_;	///* state sigma points matrix
	MatrixXd ZsigmaPoints_;	///* measurement sigma points matrix
	VectorXd weights_;		///* Weights of sigma points

	MatrixXd H_laser_;		///* state data to laser data
	MatrixXd R_laser_;		///* noise covariance matrix for laser
	MatrixXd R_radar_;		///* noise covariance matrix for radar

	int lambda_;			///* Sigma point spreading parameter

	long time_us_;				///* time when the state is true, in us
	long previous_timestamp_;	///* time of previous measurement


	double std_a_;			///* Process noise standard deviation longitudinal acceleration in m/s^2
	double std_yawdd_;		///* Process noise standard deviation yaw acceleration in rad/s^2
	double std_laspx_;		///* Laser measurement noise standard deviation position1 in m
	double std_laspy_;		///* Laser measurement noise standard deviation position2 in m 
	double std_radr_;		///* Radar measurement noise standard deviation radius in m
	double std_radphi_;		///* Radar measurement noise standard deviation angle in rad
	double std_radrd_;		///* Radar measurement noise standard deviation radius change in m/s
	double std_inf_;		///* Use that as an infinity placeholder

	double NIS_radar_;		///* the current NIS for radar
	double NIS_laser_;		///* the current NIS for laser


	UKF();			// Constructor
	virtual ~UKF();	// Destructor


					// initialization 
	void InitSensorMatrices();
	void InitStateMatrices();

	void Initialize(MeasurementPackage& m);

	// sigma points creation and mapping
	VectorXd GenerateSigmaWeights(int stateDimension, int lamdaParameter);
	void GenerateSigmaPoints(int stateDimension, int lamdaParameter);
	void PredictStateOfSigmaPoint(int sigmaPoint, double dt);
	void PredictNextState(double dt);
	void PredictNextRadarMeasurement();
	void PredictMeasurementForSigmaPoint(int sigmaPoint);


	void ProcessMeasurement(MeasurementPackage m);

	void Prediction(MeasurementPackage m);


	void Update(MeasurementPackage m);
	void UpdateLidar(MeasurementPackage m);
	void UpdateRadar(MeasurementPackage m);

	double NormalizeAngle(double angle);
	VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
};

#endif /* UKF_H */
