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

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 5;
  
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

  x_.fill(0);
  P_.fill(0.0);
  P_(0, 0) = std::pow(std_laspx_, 2);
  P_(1, 1) = std::pow(std_laspy_, 2);
  P_(2, 2) = 0.1; // the last three were defined empirically
  P_(3, 3) = 0.1;
  P_(4, 4) = 0.1;

  weights_.fill(0.5/(lambda_ + n_aug_));
  weights_(0) = lambda_/(lambda_ + n_aug_);

  x_aug.fill(0.0);
  P_aug.fill(0.0);
  P_aug(n_x_, n_x_) = pow(std_a_, 2);
  P_aug(n_x_ +1, n_x_ +1) = pow(std_yawdd_, 2);

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{

  bool debug = false;

  if (debug) {
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
  if (debug)
    std::cout << "Current timestamp: " << time_us_ / 1e6 << std::endl;

  if ((meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) || (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_))
      UpdateMeasurement(meas_package);
}

void UKF::Prediction(double delta_t)
{
    CreateAugumentedSigmaPoints();
    PredictSigmaPoints(delta_t/1e6); // divided by 1e6 to convert us to s
    PredictStateCovarianceMatrix();
}

void UKF::UpdateMeasurement(const MeasurementPackage &meas_package)
{
    SetLastMeasurementAndResetTemporaryMatrices(meas_package);
    TransformSigmaPointsToMeasurementSpace(meas_package);
    UpdateMeanPredictedMeasurement();
    UpdateCrossCorrelationAndCovarianceMatrix();
    UpdateStateVectorAndCovarianceMatrix();
    CalculateNIS(meas_package.sensor_type_);
}

void UKF::UpdateStateVectorAndCovarianceMatrix()
{
    K = Tc * S.inverse();
    z_diff = z - z_pred;
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
}

void UKF::SetLastMeasurementAndResetTemporaryMatrices(const MeasurementPackage &meas_package)
{
    z = meas_package.raw_measurements_;
    unsigned n_z = z.size();

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
        x_diff = Xsig_pred_.col(i) - x_;
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

void UKF::CreateAugumentedSigmaPoints() {
    x_aug.head(n_x_) = x_;
    P_aug.topLeftCorner(n_x_,n_x_) = P_;
    L = P_aug.llt().matrixL();
    L *= sqrt(lambda_ + n_aug_);
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; ++i) {
        Xsig_aug.col(i+1)       = x_aug + L.col(i);
        Xsig_aug.col(i+1+ n_aug_) = x_aug - L.col(i);
    }
}

void UKF::PredictSigmaPoints(double delta_t_s) {
    for (int i = 0; i< 2 * n_aug_+1; ++i) {
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        double px_p, py_p;

        if (fabs(yawd) > 0.001) {
            px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t_s) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t_s));
        } else {
            px_p = p_x + v * delta_t_s * cos(yaw);
            py_p = p_y + v * delta_t_s * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd * delta_t_s;
        double yawd_p = yawd;
        double delta_t_sq = pow(delta_t_s, 2);

        px_p = px_p + 0.5 * nu_a * delta_t_sq * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t_sq * sin(yaw);
        v_p = v_p + nu_a * delta_t_s;

        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t_s * delta_t_s;
        yawd_p = yawd_p + nu_yawdd * delta_t_s;

        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }
}

void UKF::PredictStateCovarianceMatrix() {
    x_ = Xsig_pred_ * weights_;
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        x_diff = Xsig_pred_.col(i) - x_;
        NormalizeAngle(x_diff(3));
        P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    }
}

void UKF::CalculateNIS(MeasurementPackage::SensorType sensor_type) {
    bool calculate_nis = true;

    if (calculate_nis) {
        const double radar_nis95_threshold = 7.815;
        const double lidar_nis95_threshold = 5.991;

        double nis = z_diff.transpose() * S.inverse() * z_diff;
        if (sensor_type == MeasurementPackage::LASER) {
            total_lidar_measurements++;
            if (nis < lidar_nis95_threshold)
                lidar_below_threshold++;
            double bt = lidar_below_threshold/static_cast<double>(total_lidar_measurements);
            std::cout << "NIS for LIDAR: " << nis << ". Results within 95% threshold: " << bt << std::endl;
        }
        else if (sensor_type == MeasurementPackage::RADAR) {
            total_radar_measurements++;
            if (nis < radar_nis95_threshold)
                radar_below_threshold++;
            double bt = radar_below_threshold/static_cast<double>(total_radar_measurements);
            std::cout << "NIS for RADAR: " << nis << ". Results within 95% threshold: " << bt << std::endl;
        }
    }
}