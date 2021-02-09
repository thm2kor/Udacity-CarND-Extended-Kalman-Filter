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


  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  // Initialize the state transition Matrix - F
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  noise_ax_ = 9.0; // recommended in project instructions
  noise_ay_ = 9.0; // recommended in project instructions
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
    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    previous_timestamp_ = measurement_pack.timestamp_;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // refer main.cpp for the indexing of the raw_measurements_ arrays for
      // radar and laser
      // The range, (rho), is the radial distance to the pedestrian.
      // The range is basically the magnitude of the position vector
      // which can be defined as sqrt(p_x^2 + p_y^2)
      double ro = measurement_pack.raw_measurements_[0];
      // The angle (phi) between the ray and the vehicle x direction
      // which is along the direction of movement of vehicle
      double phi = measurement_pack.raw_measurements_[1];
      // range rate rho_dot is the radial velocity is the velocity
      // along the rays
      double ro_dot = measurement_pack.raw_measurements_[2];
      // Refer to the screenshot from class ../images/radar_measurement_paremeter.png
      ekf_.x_ << ro*cos(phi), ro*sin(phi), ro_dot*cos(phi), ro_dot*sin(phi);
      cout << "EKF: First measurement for RADAR" << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // refer main.cpp for the indexing of the raw_measurements_ arrays for
      // radar and laser
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */
  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  // Set the state transition Matrix - F
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // Initialize the process covariance Matrix - Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << (pow(dt,4)/4)*noise_ax_, 0, (pow(dt,3)/2)*noise_ax_, 0,
            0, (pow(dt,4)/4)*noise_ay_, 0, (pow(dt,3)/2)*noise_ay_,
            (pow(dt,3)/2)*noise_ax_, 0, pow(dt,2)*noise_ax_, 0,
            0, (pow(dt,3)/2)*noise_ay_, 0, pow(dt,2)*noise_ay_;
  // Predict the next state
  ekf_.Predict();

  /**
   * Update
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    // Non-linear measurement function for radar. Use Extended Kalman filter
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    // The measurement update for lidar will use the regular Kalman filter
    // equations, since lidar uses linear model in the prediction step.
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
