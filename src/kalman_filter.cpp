#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using namespace std;
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
  std::cout << "Predict Start!" << std::endl;
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;  
  std::cout << "Predict Updated!" << std::endl;
  
}

void KalmanFilter::Update(const VectorXd &z) {
   std::cout << "Laser Start!" << std::endl;
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = P_* Ht * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
  std::cout << "Laser Updated!" << std::endl;
}


void KalmanFilter::UpdateEKF(const VectorXd &z) {
  std::cout << "Radar Start!" << std::endl;
  
  float rho_2 = x_(0)*x_(0) + x_(1)*x_(1); //Prediction position in normal format
  if (fabs(rho_2)<0.0001)
  {
   rho_2 = 0.0001;
  }
  float rho = sqrt(rho_2);
  float phi = atan2(x_(1), x_(0)); 
  float rho_dot =  (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  
	VectorXd y = z - z_pred;
  //Limited the output range into [-pi,pi]
  y(1) = fmod(y(1),2*pi); 
  if (y(1) < -pi) {
      y(1) = y(1) + 2 * pi;
  } else if (y(1) > pi) {
      y(1) = y(1) - 2 * pi;
  }
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = P_* Ht * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
  std::cout << "Radar Updated!" << std::endl;
}
