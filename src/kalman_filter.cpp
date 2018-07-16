#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
	P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

	VectorXd y = z - H_*x_;
	MatrixXd S = H_*P_*H_.transpose() + R_;
	MatrixXd K = P_*H_.transpose()*S.inverse();

	x_ = x_ + K*y;
	MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
	P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	float pi = 3.1415;
	float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
	float theta = atan2(x_(1), x_(0));
	float phidot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;

	while (theta - z(1) > pi/2)
		theta = theta - pi;
	while (z(1) - theta > pi/2)
		theta = theta + pi;

	VectorXd pred(3);
	pred(0) = rho;
	pred(1) = theta;
	pred(2) = phidot;

	VectorXd y = z - pred;
	MatrixXd S = H_*P_*H_.transpose() + R_;
	MatrixXd K = P_*H_.transpose()*S.inverse();

	x_ = x_ + K*y;
	MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
	P_ = (I - K*H_)*P_;
}
