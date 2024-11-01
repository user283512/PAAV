#include "tracker/KalmanFilter.h"

void KalmanFilter::init(double dt)
{
  dt_ = dt;

  // Initialize the state vector
	x_ = Eigen::VectorXd(4);

  // Initialize the state covariance matrix
  // P_ = Eigen::MatrixXd(4, 4); 
  // P_ << 100.0,  0.0,    0.0,    0.0,
  //       0.0,    100.0,  0.0,    0.0,
  //       0.0,    0.0,    100.0,  0.0,
  //       0.0,    0.0,    0.0,    100.0;
  

  // Initialize the measurement covariance matrix
  R_ = Eigen::MatrixXd(2, 2);
  R_ << 0.0225, 0.0,
        0.0,    0.0225;

  // Initialize the measurement matrix
  H_ = Eigen::MatrixXd(2, 4);
  H_ << 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0;

  // Initialize the transition matrix
  F_ = Eigen::MatrixXd(4, 4);
  F_ << 1.0, 0.0, dt_, 0.0,
        0.0, 1.0, 0.0, dt_,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;

  // Set the acceleration noise components
  double noise_ax_ = 2.;
  double noise_ay_ = 2.;

  double dt_2 = dt_ * dt_;
  double dt_3 = dt_2 * dt_;
  double dt_4 = dt_3 * dt_;

  // Initialize the process covariance matrix
  Q_ = Eigen::MatrixXd(4, 4);
  Q_ << dt_4 / 4. * noise_ax_, 0., dt_3 / 2. * noise_ax_, 0.,
        0., dt_4 / 4. * noise_ay_, 0., dt_3 / 2. * noise_ay_,
        dt_3 / 2. * noise_ax_, 0., dt_2 * noise_ax_, 0.,
        0., dt_3 / 2. * noise_ay_, 0., dt_2 * noise_ay_;
}

void KalmanFilter::predict()
{
	/**
	 * The predict() function in the Kalman filter is responsible for estimating the future state 
	 * of the system based on the current state and the transition model. 
	 * In other words, it predicts where the system will be at the next time step.
	 */

	// Status prediction: x' = F * x
	// By multiplying the current state x_ by the transition matrix F_, we obtain an estimate of the future state. 
	// The F_ matrix contains information about how the state evolves over time.
  x_ = F_ * x_;
  
	// Prediction of state covariance: P' = F * P * F^{T} + Q
	// updates the state covariance, considering both the uncertainty of the system (F_) 
	// and the uncertainty of the process (Q_).
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd &z)
{
	/**
	 * The update() function in the Kalman filter implements the correction step, 
	 * where the predicted state is updated based on the new measurement z. 
	 * The function applies the measurement to correct the current estimate and reduce the uncertainty.
	 */

	// 1. Calculation of innovation error (or residual): y = z - (H * x')
	// Calculate the difference between the z-measure and the state prediction H_ * x_. 
	Eigen::VectorXd y = z - (H_ * x_);

	// 2. Calculation of innovation covariance: S= (H * P' * H^{T}) + R
	// Calculates the innovation covariance, which represents the variance of the prediction error.
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd S = (H_ * P_ * Ht) + R_;

	// 3. Calculation of Kalman's gain: K = P' * H^{T} * S^{-1}
	// Kalman's gain determines how much new measurements affect the state estimate. 
	// A high value of K indicates higher confidence in measurements, 
	// while a low value indicates higher confidence in prediction.
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd K = P_ * Ht * Si;

  // 4. Update the estimated state: x = x' + (K * y)
	// Update the estimated state using Kalman gain and innovation.
  x_ = x_ + (K * y);
  
  // 5. Update of covariance: P = (I - K * H) * P'
	// Updates the state covariance by reducing the uncertainty according to the measurement.
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::setState(double x, double y)
{
  x_ << x, y, 0., 0.;
}