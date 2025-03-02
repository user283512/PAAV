#include "tracker/Tracklet.h"

Tracklet::Tracklet(int idTrack, double x, double y)
{
  // set id
  id_ = idTrack;

  // initialize filter
  kf_.init(0.1);
  kf_.setState(x, y);

  // set loss count to 0
  loss_count_ = 0;
}

// Predict a single measurement
void Tracklet::predict()
{
  kf_.predict();
  loss_count_++;
}

// Update with a real measurement
void Tracklet::update(double x, double y, bool lidarStatus)
{
  // measurement update
  if (!lidarStatus)
    return;

  // measurement update
  Eigen::VectorXd raw_measurements_ = Eigen::VectorXd(2);
  raw_measurements_ << x, y;
  kf_.update(raw_measurements_);
  loss_count_ = 0;
}
