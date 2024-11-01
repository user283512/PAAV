#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <eigen3/Eigen/Dense>

class KalmanFilter
{
public:
  KalmanFilter() = default;
  ~KalmanFilter() = default;

  // init the filter
  void init(double dt);

  void predict();
  void update(const Eigen::VectorXd& z);

  // setters
  void setState(double x, double y);

  // getters
  double getXCovariance() { return P_.coeff(0, 0); }
  double getYCovariance() { return P_.coeff(1, 1); }
  double getX() { return x_[0]; }
  double getY() { return x_[1]; }

private:
  // dt in seconds
  double dt_;

  /**
   * The state vector.
   * 
   * Represents the current estimated state of the system.
   * It's a 4D vector, likely representing position (x, y) and velocity (vx, vy).
   * The Kalman Filter aims to continually update this vector to better reflect the true state.
   */
  Eigen::VectorXd x_;

  /**
   * The state covariance matrix.
   * 
   * Quantifies the uncertainty in the state vector.
   * It's a 4x4 matrix, where the diagonal elements represent the variance of each state variable, 
   * and the off-diagonal elements represent the covariance between state variables.
   * 
   * A high value on a diagonal element indicates greater uncertainty on that variable, 
   * while lower values indicate greater reliability.
   */ 
  Eigen::MatrixXd P_;

  /**
   * The state transistion matrix.
   * 
   * Defines how the state vector x_ evolves from one instant of time to the next in the absence of noise.
   * This matrix models the dynamics of the system (e.g., position changes as a function of velocity) 
   * and may include terms related to time dt. 
   * 
   * Is constructed to represent a simple motion model in which the new position depends s
   * on velocity and time dt
   */ 
  Eigen::MatrixXd F_;

  /**
   * The process covariance matrix.
   * 
   * Represents the uncertainty or noise of the process model. 
   * The elements of the diagonal represent the uncertainty for each variable, 
   * and in this case are calculated as a function of dt and the noise parameters noise_ax_ and noise_ay_, 
   * which are associated with the change in acceleration.
   */ 
  Eigen::MatrixXd Q_;

  /**
   * The measurement matrix.
   * 
   * Maps the state vector x_ to the space of measurements. 
   * It transforms state variables into those we actually observe, for example 
   * by mapping (x,y,vx,vy)(x,y,vx,vy) to the measured position coordinates (x,y)(x,y). 
   * Extracts only the x and y position components, ignoring the velocities, for a partial measurement of the system.
   */ 
  Eigen::MatrixXd H_;

  /**
   * The measurement covariance matrix.
   * 
   * Represents the uncertainty associated with measurement noise. 
   * The values on the diagonal indicate the variance of the noise for each measurement 
   * (e.g., the noise of the xx and yy position measurements), while the off-diagonal elements 
   * represent the covariance between measurements. 
   * 
   * A higher value for a certain measurement indicates that the measurement itself is less reliable.
   */ 
  Eigen::MatrixXd R_;
};

#endif // KALMAN_FILTER_H_
