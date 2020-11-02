#include "ukf.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;



// Helper functions
void NormalizeAngle(double &angle) {
    while (angle > M_PI)
        angle -= 2.*M_PI;
    while (angle < -M_PI)
        angle += 2.*M_PI;
}

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 3;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  x_ = VectorXd(n_x_);
  x_.fill(0);

  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0);
  P_(0, 0) = std::pow(std_laspx_, 2);
  P_(1, 1) = std::pow(std_laspy_, 2);
  P_(2, 2) = 0.1; // the last three were defined empirically
  P_(3, 3) = 0.1;
  P_(4, 4) = 0.1;

  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);

  weights_ = VectorXd(2*n_aug_ + 1);
  weights_.fill(0.5/(lambda_ + n_aug_));
  weights_(0) = lambda_/(lambda_ + n_aug_);

  // TODO: ADD NOISE COVARIANCE MATRIX FOR BOTH LIDAR AND RADAR. ADD IT TO THE S MATRIX IN EACH CASE, MAKE SURE WHAT IT DOES
  // TODO: ADD NIS, PLAY WITH IT.

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{

  bool debug_measurements = false;

  if (debug_measurements) {
      if (meas_package.sensor_type_ == MeasurementPackage::LASER)
          std::cout << "Received a LIDAR measurement: [" << meas_package.raw_measurements_[0] << ", " << meas_package.raw_measurements_[0] << "]" << std::endl;
      if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
          std::cout << "Received a RADAR measurement: [" << meas_package.raw_measurements_[0] << ", " << meas_package.raw_measurements_[1] << ", " << meas_package.raw_measurements_[2] << "]" << std::endl;
  }

  if (!is_initialized_) {
      // initialize the position - no need to deal with RADAR, as LIDAR always comes first.
      if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
          time_us_ = meas_package.timestamp_;
          x_(0) = meas_package.raw_measurements_[0];
          x_(1) = meas_package.raw_measurements_[1];
          is_initialized_ = true;
      } else {
          std::cout << "ERROR: Unsupported position initialization measurement. " << std::endl;
      }
      return;
  }

  Prediction(meas_package.timestamp_ - time_us_);

  time_us_ = meas_package.timestamp_;

  std::cout << "Current timestamp: " << time_us_ / 1e6 << std::endl;

  if ((meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) || (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_))
      UpdateMeasurement(meas_package);
}

void UKF::Prediction(double delta_t)
{


    /**
     * TODO: Complete this function! Estimate the object's location.
     * Modify the state vector, x_. Predict sigma points, the state,
     * and the state covariance matrix.
     */


    delta_t /= 1e6; // us to s


}

void UKF::UpdateMeasurement(const MeasurementPackage &meas_package)
{
    SetLastMeasurementAndResetTemporaryMatrices(meas_package);
    TransformSigmaPointsToMeasurementSpace(meas_package);
    UpdateMeanPredictedMeasurement();
    UpdateCrossCorrelationAndCovarianceMatrix();
    UpdateStateVectorAndCovarianceMatrix();
}

void UKF::UpdateStateVectorAndCovarianceMatrix()
{
    K = Tc * S.inverse();
    z_diff = z - z_pred;
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
}

void UKF::SetLastMeasurementAndResetTemporaryMatrices(const MeasurementPackage &meas_package)
{
    unsigned n_z = meas_package.raw_measurements_.size();
    z = meas_package.raw_measurements_;

    Zsig = MatrixXd(n_z, 2 * n_aug_ + 1); // sigma points in measurement space
    Zsig.fill(0.0);

    z_pred = VectorXd(n_z); // predicted measurement
    z_pred.fill(0.0);

    S = MatrixXd(n_z, n_z); // measurement covariance matrix
    S.fill(0.0);

    Tc = MatrixXd(n_x_, n_z); // cross correlation matrix
    Tc.fill(0.0);

    R = MatrixXd(n_z, n_z); // measurement noise covariance matrix
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        R << pow(std_laspx_, 2), 0,
             0, pow(std_laspy_, 2);
    } else {
        R <<  pow(std_radr_, 2), 0, 0,
              0, pow(std_radphi_, 2), 0,
              0, 0, pow(std_radrd_, 2);
    }
}

void UKF::UpdateMeanPredictedMeasurement()
{
    z_pred = Zsig * weights_;
}

void UKF::UpdateCrossCorrelationAndCovarianceMatrix()
{
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        z_diff = Zsig.col(i) - z_pred;
        NormalizeAngle(z_diff(1));
        S += weights_(i) * z_diff * z_diff.transpose();
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        NormalizeAngle(x_diff(3));
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    S = S + R;
}

void UKF::TransformSigmaPointsToMeasurementSpace(const MeasurementPackage &meas_package)
{
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        for (int i = 0; i < 2 * n_aug_ + 1; ++i)
        {
            Zsig(0, i) = Xsig_pred_(0, i);
            Zsig(1, i) = Xsig_pred_(1, i);
        }
    } else {
        for (int i = 0; i < 2 * n_aug_ + 1; ++i)
        {
            double p_x = Xsig_pred_(0, i);
            double p_y = Xsig_pred_(1, i);
            double v = Xsig_pred_(2, i);
            double yaw = Xsig_pred_(3, i);
            double v1 = cos(yaw) * v;
            double v2 = sin(yaw) * v;

            Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);
            Zsig(1, i) = atan2(p_y, p_x);
            Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);
        }
    }
}