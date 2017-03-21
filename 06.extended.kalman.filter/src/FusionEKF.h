#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  FusionEKF();
  virtual ~FusionEKF();

  // Run the whole flow of the Kalman Filter from here.
  void ProcessMeasurement(const MeasurementPackage& measurement_pack);

  // Kalman Filter update and prediction math lives in here.
  KalmanFilter ekf_;

private:
  void Initialize(const MeasurementPackage& measurement_package);
  void Predict(const MeasurementPackage& measurement_package); // the prediction step
  void Update(const MeasurementPackage& measurement_package); // the update step
  void Update_Q_Matrix(double dt); // calculation of the Q matrix
  void Update_F_Matrix(double dt); // calculation of the F matrix
  VectorXd PredictRadarMeasurement(const VectorXd& x) const;


private:
  bool is_initialized_;
  long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  MatrixXd R_laser_;
  MatrixXd R_radar_;
  MatrixXd H_laser_;
  MatrixXd Hj_;
};

#endif /* FusionEKF_H_ */
