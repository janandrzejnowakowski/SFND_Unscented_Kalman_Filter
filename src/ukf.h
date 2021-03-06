#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using any measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateMeasurement(const MeasurementPackage &meas_package);

  void UpdateStateVectorAndCovarianceMatrix();
  void SetLastMeasurementAndResetTemporaryMatrices(const MeasurementPackage &meas_package);
  void UpdateMeanPredictedMeasurement();
  void UpdateCrossCorrelationAndCovarianceMatrix();
  void TransformSigmaPointsToMeasurementSpace(const MeasurementPackage &meas_package);
  void CreateAugumentedSigmaPoints();
  void PredictSigmaPoints(double delta_t_s);
  void PredictStateCovarianceMatrix();
  void CalculateNIS(MeasurementPackage::SensorType sensor_type);

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // time when the state is true, in us
  long long time_us_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // State dimension
  int n_x_ = 5;

  // Augmented state dimension
  int n_aug_ = n_x_ + 2;

  // Sigma point spreading parameter
  double lambda_ = 3 - n_aug_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_ = Eigen::VectorXd(n_x_);

  // state covariance matrix
  Eigen::MatrixXd P_ = Eigen::MatrixXd(n_x_, n_x_);

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_ = Eigen::MatrixXd(n_x_, 2*n_aug_ + 1);

  // Weights of sigma points
  Eigen::VectorXd weights_ = Eigen::VectorXd(2*n_aug_ + 1);

  // Temporary matrices and vectors, so that they do not need to be defined every time there is a new measurement
  Eigen::VectorXd z;
  Eigen::VectorXd z_pred;
  Eigen::VectorXd z_diff;
  Eigen::MatrixXd Zsig;
  Eigen::MatrixXd S;
  Eigen::MatrixXd Tc;
  Eigen::MatrixXd R;
  Eigen::MatrixXd K;
  Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_);
  Eigen::MatrixXd P_aug = Eigen::MatrixXd(n_aug_ , n_aug_);
  Eigen::MatrixXd L;
  Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_ , 2*n_aug_ +1);
  Eigen::VectorXd x_diff;

  unsigned total_lidar_measurements = 0;
  unsigned total_radar_measurements = 0;
  unsigned lidar_below_threshold = 0;
  unsigned radar_below_threshold = 0;
};

#endif  // UKF_H