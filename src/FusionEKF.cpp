#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
  */

  // measurement matrix H - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // NOTE: Not needed, Hj is calculated in KalmanFilter::UpdateEKF
  // measurement matrix Hj - radar
  // VectorXd x_state(4);
  // x_state << 1, 1, 0, 0;
  // Hj_ = tools.CalculateJacobian(x_state);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  float rho;
  float phi;
  float rho_dot;
  float px;
  float py;
  float vx;
  float vy;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    // state covariance matrix
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    // state transition matrix
    ekf_.F_ = MatrixXd(4, 4);

    // process covariance matrix
    ekf_.Q_ = MatrixXd(4, 4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      rho = measurement_pack.raw_measurements_[0];
      phi = measurement_pack.raw_measurements_[1];
      rho_dot = measurement_pack.raw_measurements_[2];

      px = rho * cos(phi);
      py = rho * sin(phi);
      vx = rho_dot * cos(phi);
      vy = rho_dot * sin(phi);

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];
      vx = 0;
      vy = 0;
    }

    // assign a small value to px and py when both are zero
    float epsilon = 0.000001;

    if (abs(px) <= epsilon && abs(py) <= epsilon) {
      px = epsilon;
      py = epsilon;
    }

    // initialize state
    ekf_.x_ << px, py, vx, vy;

    // initialize previous timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use sigma_ax = 9 and sigma_ay = 9 for your Q matrix.
   */
  float sigma_ax = 9;
  float sigma_ay = 9;

  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;

  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt4*sigma_ax/4, 0, dt3*sigma_ax/2, 0,
             0, dt4*sigma_ay/4, 0, dt3*sigma_ay/2,
             dt3*sigma_ax/2, 0, dt2*sigma_ax, 0,
             0, dt3*sigma_ay/2, 0, dt2*sigma_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    rho = measurement_pack.raw_measurements_[0];
    phi = measurement_pack.raw_measurements_[1];
    rho_dot = measurement_pack.raw_measurements_[2];

    VectorXd z(3);
    z << rho, phi, rho_dot;

    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(z);

  } else {
    // Laser updates
    px = measurement_pack.raw_measurements_[0];
    py = measurement_pack.raw_measurements_[1];

    VectorXd z(2);
    z << px, py;

    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
