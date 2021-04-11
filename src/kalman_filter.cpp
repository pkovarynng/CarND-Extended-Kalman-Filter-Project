#include "kalman_filter.h"
#include "math.h"

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
  cout << "KalmanFilter::Predict start" << endl;
  // state
  x_ = F_ * x_;
  
  // state covariance matrix
  P_ = F_ * P_ * F_.transpose() + Q_;
  cout << "KalmanFilter::Predict end" << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  
  VectorXd y = z - H_ * x_;
  
  // a set of calculations needed for the next two equations
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;

  MatrixXd S = H_ * PHt + R_;
  MatrixXd K = PHt * S.inverse();

  // new state
  x_ = x_ + K * y;
  
  // identity matrix for calculating P
  MatrixXd I = MatrixXd::Identity(4, 4);
  
  // new state covariance matrix
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  // convert the predictions into polar coordinates
  float rho_p = sqrt(px*px + py*py);
  float theta_p = atan2(py,px);
  float rho_dot_p = (px*vx + py*vy)/rho_p;

  VectorXd z_pred = VectorXd(3);
  z_pred << rho_p, theta_p, rho_dot_p;

  VectorXd y = z - z_pred;
  
  // normalize the angle
  while (y(1) > M_PI) {
    y(1) -= 2*M_PI;
  }
  while (y(1) < -M_PI) {
    y(1) += 2*M_PI;
  }
  
  // a set of calculations needed for the next two equations
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;

  MatrixXd S = H_ * PHt + R_;
  MatrixXd K = PHt * S.inverse();

  // new state
  x_ = x_ + K * y;
  
  // identity matrix for calculating P
  MatrixXd I = MatrixXd::Identity(4, 4);
  
  // new state covariance matrix
  P_ = (I - K * H_) * P_;
}
