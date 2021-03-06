#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
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
  VectorXd pred = VectorXd(3);

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float ro = sqrt((px*px) + (py*py));
  float ro_dot = ((px*vx) + (py*vy)) / ro;
  float theta = atan2(py,px);

  pred << ro, theta, ro_dot;

  //calculate error
  VectorXd y = z - pred;

  // normalize theta values between pi and -pi
  // idea taken over from the discussion forum
  while (y(1) < -M_PI) {
    y(1) = y(1) + 2.0*M_PI;
  }
  while (y(1) > M_PI) {
    y(1) = y(1) - 2.0*M_PI;
  }

  // projecting the system uncertainty into the measurement space + meas noise
  MatrixXd S = (H_ * P_ * H_.transpose()) + R_;
  // calculate Kalman Gain
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  // update positions and covariance matrix
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - (K * H_)) * P_;

}
