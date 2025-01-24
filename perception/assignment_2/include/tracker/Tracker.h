#ifndef TRACKER_H_
#define TRACKER_H_

#include "tracker/Tracklet.h"
#include <limits>

// The Tracker class implements a Multi-Target Tracking Management system.
// It manages a dynamic set of tracked objects, represented as Tracklets,
// and is responsible for predicting, updating, creating and removing tracks based on observations from sensors.
class Tracker
{
public:
  Tracker(
      double distance_threshold,
      double covariance_threshold,
      int loss_threshold)
      : cur_id_{0},
        distance_threshold_{distance_threshold},
        covariance_threshold_{covariance_threshold},
        loss_threshold_{loss_threshold},
        tracks_{},
        associated_track_det_ids_{}
  {
  }

  ~Tracker() = default;

  // This method is responsible for removing tracklets from the list when they are no longer valid.
  // The validity of a tracklet is determined based on criteria they consider:
  // - the number of consecutive frames in which the tracklet was not associated with any observation (loss_count).
  // - the uncertainty (covariance) of its X and Y coordinates.
  void removeTracks();

  // This method is responsible for creating new tracklets for detections not associated with existing tracks.
  // It is used in the tracking process to handle new objects detected in the scene that do not
  // correspond to any active track.
  // Input:
  //  - a vector of Boolean values indicating, for each detection, whether it has been associated with
  //    an existing track (true) or not (false).
  //  - vectors containing the X and Y coordinates of the centers of the surveys.
  void addTracks(const std::vector<bool> &associated_detections,
                 const std::vector<double> &centroids_x,
                 const std::vector<double> &centroids_y);

  // This method is responsible for associating detections (the coordinates of centroids_x and centroids_y)
  // with existing tracks (Tracklets) based on the distance between them.
  // This process makes it possible to update the position of active tracklets in the system by linking
  // them to the nearest detections detected by the sensor.
  // Objective:
  //  - the system must decide which track to assign each detection to, or whether a detection represents
  //    a new object not yet monitored.
  // Input:
  //  - a vector that keeps track of which detections have already been associated with a track.
  //    If a detection has already been used, it is marked as true.
  //  - the coordinates of the centers of detections in the scene (e.g., the positions of detected people or objects).
  // Output:
  //  - updates the vector to indicate which detections have been associated with a track.
  void dataAssociation(std::vector<bool> &associated_detections,
                       const std::vector<double> &centroids_x,
                       const std::vector<double> &centroids_y);

  void track(const std::vector<double> &centroids_x,
             const std::vector<double> &centroids_y,
             bool lidarStatus);

  // getters
  const std::vector<Tracklet> &getTracks() { return tracks_; }

private:
  // A list of tracklets
  std::vector<Tracklet> tracks_;
  // A unique ID for each tracklet
  int cur_id_;

  // A list of pairs that associates tracklet ID with detection ID.
  std::vector<std::pair<int, int>> associated_track_det_ids_;

  // A threshold for determining whether a detection is close enough to an existing tracklet
  // to be considered part of it.
  double distance_threshold_;

  // A threshold for determining whether the uncertainty in the measurements is small enough
  // to make a correct association.
  double covariance_threshold_;

  // A threshold for determining when a tracklet is considered lost if it is no longer
  // associated with detections for a certain number of frames.
  int loss_threshold_;
};

#endif // TRACKER_H_
