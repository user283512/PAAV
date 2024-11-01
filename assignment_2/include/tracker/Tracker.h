#ifndef TRACKER_H_
#define TRACKER_H_

#include "tracker/Tracklet.h"
#include <limits>

class Tracker
{
public:
  Tracker();
  ~Tracker() = default;

  // handle tracklets
  void removeTracks();
  void addTracks(const std::vector<bool> &associated_detections,
                 const std::vector<double> &centroids_x,
                 const std::vector<double> &centroids_y);

  // associate tracklets and detections
  void dataAssociation(std::vector<bool> &associated_detections,
                       const std::vector<double> &centroids_x,
                       const std::vector<double> &centroids_y);

  // track objects
  void track(const std::vector<double> &centroids_x,
             const std::vector<double> &centroids_y,
             bool lidarStatus);

  // getters
  const std::vector<Tracklet> &getTracks() { return tracks_; }

private:
  /**
   * This vector contains all Tracklet objects currently managed by the Tracker. 
   * Each Tracklet represents a single track, i.e., a detected object (e.g., a vehicle, a person, etc.) 
   * that is tracked over time.
   * 
   * Each Tracklet includes a Kalman filter to maintain an estimate of the state of the object 
   * (position, velocity, etc.) and to predict future movement of the object.
   * During each update, Tracklets in tracks_ are associated with new detections to update 
   * their position and reduce uncertainty through prediction and correction steps.
   */
  std::vector<Tracklet> tracks_;

  /**
   * Rappresents the current ID to be assigned to new Tracklets. This unique ID is used to identify 
   * and track each object individually. 
   * Each time a new Tracklet is created (e.g., when an object is detected but not associated 
   * with an existing Tracklet), cur_id_ is incremented to ensure that each Tracklet has a unique ID.
   */
  int cur_id_;

  /**
   * This vector of pairs maintains associations between existing Tracklets and detections (detections) 
   * in a given update. Each element represents a pair where:
   * - track_id is the index of the Tracklet in tracks_,
   * - detection_id is the index of the current detection in a list of provided detections.
   * 
   * This vector aids in the management of matches between tracks and detections, 
   * allowing Tracklets to be updated correctly with new sensor data.
   */
  std::vector<std::pair<int, int>> associated_track_det_ids_;

  /**
   * Is a distance threshold used in the association process between Tracklets and new detections. 
   * If the distance between a Tracklet and a detection is less than this threshold, 
   * the detection can be considered as a possible association to the Tracklet. 
   * This parameter is used to avoid unlikely associations, improving the robustness of tracking.
   */
  double distance_threshold_;

  /**
   * Represents a threshold for covariance, used to filter out or remove Tracklets with 
   * a high level of uncertainty.
   * If the covariance of a Tracklet exceeds this threshold, it means that the Kalman filter 
   * has accumulated too much uncertainty on that Tracklet. 
   * In this case, the Tracklet could be considered unreliable and could be removed.
   */  
  double covariance_threshold;

  /**
   * Represents the maximum number of consecutive updates a Tracklet can miss before being 
   * removed from the Tracker. 
   * Each Tracklet has an internal counter (loss_count_) that increments each time it cannot be 
   * associated with a new detection. If this counter exceeds loss_threshold, the Tracklet is deleted 
   * because it is assumed that the Tracklet object is no longer present or can no longer be detected.
   */
  int loss_threshold;
};

#endif // TRACKER_H_
