#include "kalman_filter.h"

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
  cout << "KalmanFilter::Update" << endl;
  // calculate error
  VectorXd y = z - (H_ * x_);
  cout << "y:" << y << endl;
  // projecting the system uncertainty into the measurement space + meas noise
  MatrixXd S = (H_ * P_ * H_.transpose()) + R_;
  cout << "S:" << S << endl;
  // calculate Kalman Gain
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  cout << "K:" << K << endl;
  // update positions and covariance matrix
  x_ = x_ + (K * y);
  cout << "new x_:" << x_ << endl;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - (K * H_)) * P_;
  cout << "new P_:" << P_ << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd h = VectorXd(3);

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  double ro = sqrt((px*px) + (py*py));
  double ro_dot = ((px*vx) + (py*vy)) / ro;
  double theta = atan2(py,px);

  h << ro, theta, ro_dot;

  //calculate error
  VectorXd y = z - h;

  // normalize theta between pi and -pi
  // logic suggestion from mentors
  while (y(1) < - M_PI ){
    y(1) = y(1) + 2.0*M_PI ;
  }
  while(y(1) > M_PI ){
    y(1) = y(1) - 2.0*M_PI ;
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
