#include "tracker/Tracker.h"
#include <iostream>

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
	if (tracks_.empty()) 
		return;

	// We invert the tracks_ vector to avoid iterator invalidation problems when we delete elements.
  for (size_t i = tracks_.size(); i > 0; i--)
	{
		auto& track = tracks_[i - 1];
    if (track.getLossCount() > loss_threshold || 
				track.getXCovariance() > covariance_threshold || 
				track.getYCovariance() > covariance_threshold)
		{
			std::cout << "Removing Tracklet ID " << track.getId() 
								<< " due to excessive loss count or uncertainty." 
								<< std::endl;
      tracks_.erase(tracks_.begin() + (i - 1));
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
	{
		if (!associated_detections[i])
		{
			std::cout << "Creating new Tracklet for detection at (" 
								<< centroids_x[i] << ", " << centroids_y[i] << ")" 
								<< std::endl;
			tracks_.emplace_back(cur_id_++, centroids_x[i], centroids_y[i]);
		}
	}
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
	}
}

void Tracker::track(
	const std::vector<double> &centroids_x,
  const std::vector<double> &centroids_y,
  bool lidarStatus
)
{
	std::cout << "Starting tracking cycle..." << std::endl;

	// ======================================
	// 1. Prediction
	// ======================================
	//
	// Before associating new detections, each existing tracklet calls the predict method 
	// to update the predicted state.
	// This helps estimate where the tracklets will be at the next round of detections.
	for (Tracklet& track : tracks_)
	{
		std::cout << "Tracklet ID " << track.getId() 
							<< " pre-prediction state: (" << track.getX() << ", " << track.getY() << ")"
							<< std::endl;
		track.predict();
		std::cout << "Tracklet ID " << track.getId() 
							<< " post-prediction state: (" << track.getX() << ", " << track.getY() << ")"
							<< std::endl;
	}
	
	// ======================================
	// 2. Data Association
	// ======================================
	// 
	// Calls the dataAssociation method to associate new centroids (detections) with existing tracklets.
	// dataAssociation updates associated_detections to indicate which detections have been associated.
	std::vector<bool> associated_detections;
	dataAssociation(associated_detections, centroids_x, centroids_y);
	for (size_t i = 0; i < tracks_.size(); i++) 
	{ 
		std::cout << "Tracklet ID " << tracks_[i].getId() << " is associated with detection: " 
							<< (associated_detections[i] ? "Yes" : "No") 
							<< std::endl; 
	}

	// ======================================
	// 3. Addition of New Tracklets
	// ======================================
	//
	// Use the addTracks method to create new tracklets for detections that have not been associated 
	// with any existing tracklet.
	// Each new unassociated detection generates a new tracklet with a unique ID.
	addTracks(associated_detections, centroids_x, centroids_y);

	// ======================================
	// 4. Addition of New Tracklets
	// ======================================
	// Call the removeTracks method to remove tracklets that have too many update failures 
	// (exceed loss_threshold) or too much uncertainty (exceed covariance_threshold).
	removeTracks();

	std::cout << "Tracking cycle completed." << std::endl;
}
