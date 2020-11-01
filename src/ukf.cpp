#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
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
  P_ = MatrixXd::Identity(n_x_, n_x_);

  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);

  weights_ = VectorXd(2*n_aug_ + 1);
  weights_.fill(0.5/(lambda_ + n_aug_));
  weights_(0) = lambda_/(lambda_ + n_aug_);

  // TODO: ADD NOISE COVARIANCE MATRIX FOR BOTH LIDAR AND RADAR. ADD IT TO THE S MATRIX IN EACH CASE, MAKE SURE WHAT IT DOES
  // TODO: ADD NIS, PLAY WITH IT.

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

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
          std::cout << "Unsupported position initialization measurement. " << std::endl;
          throw std::exception(); // In case I missed something
      }
      return;
  }

  Prediction(meas_package.timestamp_ - time_us_);

  time_us_ = meas_package.timestamp_;

  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
      UpdateLidar(meas_package);
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
      UpdateRadar(meas_package);
}

void UKF::Prediction(double delta_t) {


    /**
     * TODO: Complete this function! Estimate the object's location.
     * Modify the state vector, x_. Predict sigma points, the state,
     * and the state covariance matrix.
     */


    delta_t /= 1e6; // us to s


}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}