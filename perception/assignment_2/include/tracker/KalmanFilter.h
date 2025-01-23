#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <eigen3/Eigen/Dense>

// Objective:
// - The Kalman filter is used to track the position of objects over time.
//   This means that given a set of noisy measurements of an object's position,
//   the Kalman filter can estimate the current position and predict the future position of the object.
// How it works:
// - Prediction: Uses the state transition matrix to calculate the predicted position and velocity of the object.
// - Update: Uses the new measurement to correct the predicted estimate, reducing the uncertainty.
class KalmanFilter
{
public:
  KalmanFilter() = default;
  ~KalmanFilter() = default;

  // Initialize the filter with the given time step (dt).
  void init(double dt);

  // Perform the prediction step of the Kalman Filter, updating the state and covariance matrix.
  void predict();

  // Perform the update step of the Kalman Filter using the provided measurement vector z.
  // This adjusts the predicted state based on the measurement.
  void update(const Eigen::VectorXd &z);

  // Set the state to the given x and y coordinates.
  // Useful for initializing or resetting the filter's state.
  void setState(double x, double y);

  // Getters to retrieve specific state or covariance information.
  double getXCovariance() { return P_.coeff(0, 0); }
  double getYCovariance() { return P_.coeff(1, 1); }
  double getX() { return x_[0]; }
  double getY() { return x_[1]; }

private:
  // Time step in seconds, used for calculating the state transition matrix (F_).
  double dt_;

  // Represents the state vector, and contains position and velocity information [x, y, vx, vy]
  Eigen::VectorXd x_;

  // State covariance matrix, representing the uncertainty of the state estimate.
  Eigen::MatrixXd P_;

  // State transition matrix, used to model how the state evolves over time.
  Eigen::MatrixXd F_;

  // Process noise covariance matrix, accounting for uncertainty in the process model.
  Eigen::MatrixXd Q_;

  // Measurement matrix, mapping the state to the measurement space.
  Eigen::MatrixXd H_;

  // Measurement noise covariance matrix, representing the uncertainty in the measurements.
  Eigen::MatrixXd R_;
};

#endif // KALMAN_FILTER_H_
