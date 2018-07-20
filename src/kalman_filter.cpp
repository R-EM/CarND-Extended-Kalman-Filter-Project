#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}
void NormaliseAngle(double& phi);

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
	
	MatrixXd PHt = P_*H_.transpose();

	VectorXd y = z - H_*x_;
	MatrixXd S = H_*PHt + R_;
	MatrixXd K = PHt*S.inverse();

	x_ = x_ + (K*y);
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
	float phidot = 0.0001;// = (x_(0)*x_(2) + x_(1)*x_(3))/rho;

	// If px or py are very small, theta and phidot are set to zero
	if (fabs(rho) >= 0.0001)// || fabs(x_(1)) < 0.0001)
		phidot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;


	VectorXd pred(3);
	pred(0) = rho;
	pred(1) = theta;
	pred(2) = phidot;

	/*
	if (pred(1) - z(1) > pi/2)
		pred(1) = theta - pi;
	if (z(1) - pred(1) > pi/2)
		pred(1) = theta + pi;
	*/

	//pred(1) = atan2(sin(theta), cos(theta));

	
	MatrixXd PHt = P_*H_.transpose();

	VectorXd y = z - pred;
	MatrixXd S = H_*PHt + R_;
	MatrixXd K = PHt*S.inverse();

	NormaliseAngle(y(1));

	x_ = x_ + K*y;
	MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
	P_ = (I - K*H_)*P_;
}

void NormaliseAngle(double& phi)
{
	phi = atan2(sin(phi), cos(phi));
}