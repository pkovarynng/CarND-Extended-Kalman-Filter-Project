#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  
  // create a 4D state vector, we don't know yet the values of the x state
  ekf_.x_ = VectorXd(4);

  // the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;  
  
  // state covariance matrix P
  ekf_.P_ = MatrixXd(4,4);
  
  // noise
  ekf_.Q_ = MatrixXd(4,4);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // set the state with the initial location and zero velocity
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
      			measurement_pack.raw_measurements_[1], 
        	    0, 
                0;
    }
    
    // set the timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float const dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  
  // update previous timestamp
  previous_timestamp_ = measurement_pack.timestamp_;

  // modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;  

  // set the process noise covariance matrix Q
  
  // some pre-computations first
  float const dt2 = dt*dt;
  float const dt3 = dt2*dt;
  float const dt4 = dt3*dt;   
  
  // set the acceleration noise components
  float const noise_ax = 9;
  float const noise_ay = 9;
  
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
	     0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
	     dt3/2*noise_ax, 0, dt2*noise_ax, 0,
	     0, dt3/2*noise_ay, 0, dt2*noise_ay;
  
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates

  } else {
    // TODO: Laser updates

    // use the Laser H matrix
    ekf_.H_ = H_laser_;

    // use the Laser R matrix
    ekf_.R_ = MatrixXd(2,2);
    ekf_.R_ = R_laser_;
    
    // update
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
