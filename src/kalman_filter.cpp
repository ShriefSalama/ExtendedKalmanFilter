#include "kalman_filter.h"
#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

Tools tools;

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  ///////////////////////////////////////////////////
  MatrixXd H_input = H_;

  MatrixXd y = H_input * x_;
  ///////////////////////////////////////////////////

  MatrixXd s = H_input * P_ * H_input.transpose() + R_ ;
  MatrixXd k = P_ * H_input.transpose() * s.inverse();
  MatrixXd z_pred = z - y ;

  //new state
  x_ = x_ + (k*z_pred);
  P_ = (I - k* H_input) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  ////////////////////////////////////////////////////
		// Extended Functions //
  ////////////////////////////////////////////////////

  MatrixXd Hj = tools.CalculateJacobian(x_);
  MatrixXd H_input = Hj;

  VectorXd y(3);

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float c4 = sqrt(px*px + py*py);

  y << c4, atan2(py,px) , (px*vx+py*vy)/c4;
  VectorXd z_pred = z - y ;

  while (z_pred(1) > M_PI || z_pred(1) < -M_PI ) {
	if (z_pred(1) > M_PI) z_pred(1) -= 2*M_PI;
	else if (z_pred(1) < -M_PI) z_pred(1) += 2*M_PI;
	}

  ///////////////////////////////////////////////////


  MatrixXd s = (H_input * P_ * H_input.transpose()) + R_ ;
  MatrixXd k = P_ * H_input.transpose() * s.inverse();


  //new state
  x_ = x_ + (k*z_pred);
  P_ = (I - k* H_input) * P_;


}
