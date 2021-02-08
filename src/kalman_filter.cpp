#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  //New position Prediction = State_Transition_Matrix*Prev_Position + Noise
  x_ = F_ * x_;
  //state covariance matrix update equation
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // calculate error
  VectorXd y = z - (H_ * x_);
  // projecting the system uncertainty into the measurement space + meas noise
  MatrixXd S = (H_ * P_ * H_.transpose()) + R_;
  // calculate Kalman Gain
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  // update positions and covariance matrix
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - (K * H_)) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
