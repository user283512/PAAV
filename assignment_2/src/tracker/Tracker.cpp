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
	// We invert the tracks_ vector to avoid iterator invalidation problems when we delete elements.
  for (size_t i = tracks_.size() - 1; i >= 0; i--)
	{
		auto& track = tracks_[i];
    if (
			track.getLossCount() > loss_threshold || 
			track.getXCovariance() > covariance_threshold || 
			track.getYCovariance() > covariance_threshold
		)
		{
      tracks_.erase(tracks_.begin() + i);
    }
  }
}

void Tracker::addTracks(
	const std::vector<bool> 	&associated_detections, 
	const std::vector<double> &centroids_x, 
	const std::vector<double> &centroids_y
)
{
	// For each detection, we check associated_detections[i]. 
	// If it is false, it means that the detection is not associated with any existing track, 
	// so a new Tracklet will be created.
	for (size_t i = 0; i < associated_detections.size(); i++)
		if (!associated_detections[i])
			tracks_.emplace_back(cur_id_++, centroids_x[i], centroids_y[i]);
}


void Tracker::dataAssociation(
	std::vector<bool> 				&associated_detections, 
	const std::vector<double> &centroids_x, 
	const std::vector<double> &centroids_y
)
{
	associated_detections.assign(centroids_x.size(), false); 
	associated_track_det_ids_.clear();

	// For each tracklet, search for the nearest detection not yet associated.
	for (size_t i = 0; i < tracks_.size(); i++) 
	{
		Tracklet &track = tracks_[i]; 
		
		// Skip tracklets with too much uncertainty
		if (track.getXCovariance() > covariance_threshold || 
				track.getYCovariance() > covariance_threshold)
		{
			continue; 
		}

		double min_distance = std::numeric_limits<double>::max(); 
		int best_match = -1;

		// For each detection
		for (size_t j = 0; j < centroids_x.size(); ++j) 
		{
			if (associated_detections[j]) 
				continue;

			// The Euclidean distance is calculated and compared with the distance_threshold_. 
			// If the detection is close enough and has the smallest distance found so far, 
			// it is considered as the "best match"
			double dx = centroids_x[j] - track.getX(); 
			double dy = centroids_y[j] - track.getY(); 
			double distance = std::sqrt(dx * dx + dy * dy); 
			if (distance < min_distance && distance < distance_threshold_) 
			{ 
				min_distance = distance; 
				best_match = j; 
			}
		}

		// If best_match is found (i.e., a detection meets the distance criterion)
		if (best_match != -1) 
		{ 
			// Mark as associated in associated_detections
			associated_detections[best_match] = true; 

			// Save in associated_track_det_ids_ to update current associations.
			associated_track_det_ids_.emplace_back(i, best_match); 

			// Call track.update() with the detection coordinates to update the tracklet with the new location.
			track.update(centroids_x[best_match], centroids_y[best_match], true);
		}
		else
		{
			// No association found; increments the tracklet loss counter
			track.incrementLossCount();
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
