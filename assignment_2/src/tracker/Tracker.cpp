#include "tracker/Tracker.h"

Tracker::Tracker()
	: cur_id_{ 0 },
		distance_threshold_{ 30.0f },		// the maximum distance for which a detection can be considered 
																		// to be associated with an existing Tracklet.
																		// A value of 30.0 meters may be reasonable for tracking objects 
																		// at short or medium range (e.g., vehicles or pedestrians).
		
		covariance_threshold{ 500.0f }, // The covariance threshold is used to assess the quality of the estimate. 
																		// A value of 500.0 may indicate a moderate amount of uncertainty, 
																		// which is acceptable. 
																		// If the covariance of a Tracklet exceeds this value, it may be considered 
																		// unreliable and therefore removed.

		loss_threshold{ 3 },						// This value defines how many consecutive updates a Tracklet can "miss"
																		// (i.e., not find an association) before it is removed.
																		// An object may be undetected for three consecutive updates before it is 
																		// removed from tracking
		
		tracks_{},
		associated_track_det_ids_{}
{}


void Tracker::removeTracks()
{
	/**
	 * This function iterate through the vector of Tracklets and removes those with too many leaks or 
	 * too much uncertainty, keeping only those Tracklets active and reliable for the next tracking cycle.
	 */
	
	// We invert the tracks_ vector to avoid iterator invalidation problems when we delete elements.
  for (int i = tracks_.size() - 1; i >= 0; i--)
	{
		auto& track = tracks_[i];
    if (
			track.getLossCount() > loss_threshold_ || 
			track.getXCovariance() > covariance_threshold || 
			track.getYCovariance() > covariance_threshold
		)
		{
      tracks_.erase(tracks_.begin() + i);
    }
  }
}

/** This function add new tracks to the set of tracks ("tracks_" is the object that contains this) */
void Tracker::addTracks(
	const std::vector<bool> 	&associated_detections, 
	const std::vector<double> &centroids_x, 
	const std::vector<double> &centroids_y
)
{
	// Adding not associated detections
	for (size_t i = 0; i < associated_detections.size(); i++)
		if (!associated_detections[i])
			tracks_.emplace_back(cur_id_++, centroids_x[i], centroids_y[i]);
}

/**
 * This function associates detections (centroids_x,centroids_y) with the tracks (tracks_)
 * Input:
 * 	+associated_detection an empty vector to host the associated detection
 * 	centroids_x & centroids_y measurements representing the detected objects
 */
void Tracker::dataAssociation(
	std::vector<bool> 				&associated_detections, 
	const std::vector<double> &centroids_x, 
	const std::vector<double> &centroids_y
)
{
	//Remind this vector contains a pair of tracks and its corresponding
	associated_track_det_ids_.clear();

	for (size_t i = 0; i < tracks_.size(); i++)
	{
		int closest_point_id = -1;
		double min_dist = std::numeric_limits<double>::max();

		for (size_t j = 0; j < centroids_x.size(); j++)
		{
			// Find the closest detection (centroids_x, centroids_y) to the current track
			if (!associated_detections[j])
			{
				double dx = tracks_[i].getX() - centroids_x[j];
        double dy = tracks_[i].getY() - centroids_y[j];
        double dist = std::sqrt(dx * dx + dy * dy);
				if (dist < min_dist) 
				{
          min_dist = dist;
          closest_point_id = j;
				}
			}
		}

		// Associate the closest detection to a tracklet
		if (min_dist < distance_threshold_ && closest_point_id != -1)
		{
			associated_track_det_ids_.emplace_back(closest_point_id, i);
			associated_detections.at(closest_point_id) = true;
		}
	}
}

void Tracker::track(
	const std::vector<double> &centroids_x,
  const std::vector<double> &centroids_y,
  bool lidarStatus
)
{
	std::vector<bool> associated_detections(centroids_x.size(), false);

	/**
	 * ======================================
	 * 1. Prediction
	 * ======================================
	 */

	// Predict the position:
	// for each track --> Predict the position of the tracklets
	for (Tracklet& track : tracks_)
    track.predict();
	
	/**
	 * ======================================
	 * 2. Association
	 * ======================================
	 */

	// Associate the predictions with the detections
	dataAssociation(associated_detections, centroids_x, centroids_y);

	/**
	 * ======================================
	 * 3. Update
	 * ======================================
	 */

	// Update tracklets with the new detections
	for (const auto& pair : associated_track_det_ids_)
	{
		int det_id = pair.first;
		int track_id = pair.second;
		tracks_.at(track_id).update(centroids_x.at(det_id), centroids_y.at(det_id), lidarStatus);
	}

	/**
	 * ======================================
	 * 4. Removal
	 * ======================================
	 */

	// Remove dead traces (with loss above threshold)
	removeTracks();

	/**
	 * ======================================
	 * 5. Addition
	 * ======================================
	 */

	// Add new tracklets
	addTracks(associated_detections, centroids_x, centroids_y);
}
