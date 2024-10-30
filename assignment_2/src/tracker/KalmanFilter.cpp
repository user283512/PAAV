#include "tracker/KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::init(double dt)
{
  dt_ = dt;

  // create a 4D state vector
  x_ = Eigen::VectorXd(4);

  // In this update, the state covariance matrix P_ is initialized with values 
  // representing a large uncertainty for the initial velocities (1000), 
  // while the initial positions are assigned a value of
  P_ = Eigen::MatrixXd(4, 4); 
  P_ << 1.0, 0.0, 0.0,    0.0,
        0.0, 1.0, 0.0,    0.0,
        0.0, 0.0, 1000.0, 0.0,
        0.0, 0.0, 0.0,    1000.0;

  // measurement covariance
  R_ = Eigen::MatrixXd(2, 2);
  R_ << 0.0225, 0.,
      0., 0.0225;

  // measurement matrix
  H_ = Eigen::MatrixXd(2, 4);
  H_ << 1., 0., 0., 0.,
      0., 1., 0., 0.;

  // the transition matrix F
  F_ = Eigen::MatrixXd(4, 4);
  F_ << 1., 0., dt_, 0.,
      0., 1., 0., dt_,
      0., 0., 1., 0.,
      0., 0., 0., 1.;

  // set the acceleration noise components
  double noise_ax_ = 2.;
  double noise_ay_ = 2.;

  double dt_2 = dt_ * dt_;
  double dt_3 = dt_2 * dt_;
  double dt_4 = dt_3 * dt_;

  // set the process covariance matrix Q
  Q_ = Eigen::MatrixXd(4, 4);
  Q_ << dt_4 / 4. * noise_ax_, 0., dt_3 / 2. * noise_ax_, 0.,
      0., dt_4 / 4. * noise_ay_, 0., dt_3 / 2. * noise_ay_,
      dt_3 / 2. * noise_ax_, 0., dt_2 * noise_ax_, 0.,
      0., dt_3 / 2. * noise_ay_, 0., dt_2 * noise_ay_;
}

void KalmanFilter::predict()
{
  // Project the state ahead
  x_ = F_ * x_;
  // Project the error covariance ahead
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd &z)
{
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd S = H_ * P_ * Ht + R_;  // Covariance of the measure
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd K = P_ * Ht * Si;       // Kalman gain

  // Update the estimated state
  Eigen::VectorXd y = z - H_ * x_;
  x_ = x_ + K * y;
  
  // Update the error covariance
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::setState(double x, double y)
{
  x_ << x, y, 0., 0.;
}