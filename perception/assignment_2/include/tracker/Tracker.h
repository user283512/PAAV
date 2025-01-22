#ifndef TRACKER_H_
#define TRACKER_H_

#include "tracker/Tracklet.h"
#include <limits>

class Tracker
{
public:
  Tracker()
      : cur_id_{0},
        distance_threshold_{30.0f}, // the maximum distance for which a detection can be considered
                                    // to be associated with an existing Tracklet.
                                    // A value of 30.0 meters may be reasonable for tracking objects
                                    // at short or medium range (e.g., vehicles or pedestrians).

        covariance_threshold{500.0f}, // The covariance threshold is used to assess the quality of the estimate.
                                      // A value of 500.0 may indicate a moderate amount of uncertainty,
                                      // which is acceptable.
                                      // If the covariance of a Tracklet exceeds this value, it may be considered
                                      // unreliable and therefore removed.

        loss_threshold{3}, // This value defines how many consecutive updates a Tracklet can "miss"
                           // (i.e., not find an association) before it is removed.
                           // An object may be undetected for three consecutive updates before it is
                           // removed from tracking

        tracks_{},
        associated_track_det_ids_{}
  {
  }
  ~Tracker() = default;

  // Removes tracklets that are no longer valid or have been lost for a long period of time.
  void removeTracks();

  // Adds new tracklets based on associated detections
  void addTracks(
      const std::vector<bool> &associated_detections,
      const std::vector<double> &centroids_x,
      const std::vector<double> &centroids_y);

  // Associates detections (centroids_x, centroids_y) with the tracks (tracks_)
  void dataAssociation(
      std::vector<bool> &associated_detections,
      const std::vector<double> &centroids_x,
      const std::vector<double> &centroids_y);

  // The main function that performs tracking, predicts the location of existing
  // tracklets and updates their location based on current detections.
  void track(
      const std::vector<double> &centroids_x,
      const std::vector<double> &centroids_y,
      bool lidarStatus);

  // getters
  const std::vector<Tracklet> &getTracks() { return tracks_; }

private:
  // A list of tracklets
  std::vector<Tracklet> tracks_;
  // A unique ID for each tracklet
  int cur_id_;

  // A list of pairs that associates tracklet IDs with survey IDs.
  std::vector<std::pair<int, int>> associated_track_det_ids_;

  // A threshold for determining whether a detection is close enough to an existing tracklet
  // to be considered part of it.
  double distance_threshold_;

  // A threshold for determining whether the uncertainty in the measurements is small enough
  // to make a correct association.
  double covariance_threshold;

  // A threshold for determining when a tracklet is considered lost if it is no longer
  // associated with detections for a certain number of frames.
  int loss_threshold;
};

#endif // TRACKER_H_
