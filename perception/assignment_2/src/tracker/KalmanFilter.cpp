#include "tracker/KalmanFilter.h"

void KalmanFilter::init(double dt)
{
  dt_ = dt;

  // Initial position (0,0) and initial speed (0,0)
  constexpr float pos_x = 0.0f;
  constexpr float pos_y = 0.0f;
  constexpr float vel_x = 0.0f;
  constexpr float vel_y = 0.0f;
  x_ = Eigen::VectorXd(4);
  x_ << pos_x, pos_y, vel_x, vel_y;

  // This matrix represents the initial uncertainty on the state variables
  constexpr float pos_variance = 1.0f; // Variance for position
  constexpr float vel_variance = 1.0f; // Variance for velocity
  P_ = Eigen::MatrixXd(4, 4);
  P_ << pos_variance, 0.0, 0.0, 0.0,
      0.0, pos_variance, 0.0, 0.0,
      0.0, 0.0, vel_variance, 0.0,
      0.0, 0.0, 0.0, vel_variance;

  // The measurement noise covariance matrix R_ represents the uncertainty of the sensor system or observed measurements.
  // A variance of 0.0225 indicates low uncertainty in measurements (the sensor is considered quite accurate).
  constexpr float measurement_noise_x = 0.0225;
  constexpr float measurement_noise_y = 0.0225;
  R_ = Eigen::MatrixXd(2, 2);
  R_ << measurement_noise_x, 0.0,
      0.0, measurement_noise_y;

  // The measurement matrix H_ is used to transform the state vector x_ into the space of measurements.
  // In other words, it maps the complete state [x, y, vx, vy] onto the observable variables,
  // which in this case are only the x and y positions.
  H_ = Eigen::MatrixXd(2, 4);
  H_ << 1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0;

  // The state transition matrix F_ describes how the state vector evolves over time
  // according to the dynamic model of the system:
  // pos_x_new ​= pos_x_old ​+ vel_x*dt
  // pos_y_new ​= pos_y_old ​+ vel_y*dt
  F_ = Eigen::MatrixXd(4, 4);
  F_ << 1.0, 0.0, dt_, 0.0,
      0.0, 1.0, 0.0, dt_,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  // set the acceleration noise components
  constexpr float noise_ax_ = 2.0;
  constexpr float noise_ay_ = 2.0;
  double dt_2 = pow(dt_, 2);
  double dt_3 = pow(dt_, 3);
  double dt_4 = pow(dt_, 4);

  // The process noise covariance matrix Q_ represents the uncertainty associated with
  // the state transition model (the F_ matrix).
  // The model assumes that the equation of motion is:
  // x(t) = x0​ + (v0 ​* t) + (1/2 * ​a * t)
  // v(t) = v0 ​+ (a * t)
  Q_ = Eigen::MatrixXd(4, 4);
  Q_ << dt_4 / 4.0 * noise_ax_, 0.0, dt_3 / 2.0 * noise_ax_, 0.0,
      0.0, dt_4 / 4.0 * noise_ay_, 0.0, dt_3 / 2.0 * noise_ay_,
      dt_3 / 2.0 * noise_ax_, 0.0, dt_2 * noise_ax_, 0.0,
      0.0, dt_3 / 2.0 * noise_ay_, 0.0, dt_2 * noise_ay_;
}

void KalmanFilter::predict()
{
  // State prediction: x_ = F_ * x_
  x_ = F_ * x_;
  // State covariance prediction: P_ = (F_ * P_ * F^T) + Q_
  P_ = (F_ * P_ * F_.transpose()) + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd &z)
{
  // Calculate the innovation y = z - H * x_
  Eigen::VectorXd y = z - H_ * x_;
  // Calculate the innovation covariance matrix S = H * P * H^T + R
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  // Calculate the Kalman gain K = P * H^T * S^-1
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  // New estimate of the state x_ = x_ + K * y
  x_ = x_ + K * y;
  // Identity matrix
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
  // Update the covariance P = (I - K * H) * P
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::setState(double x, double y)
{
  x_ << x, y, 0., 0.;
}