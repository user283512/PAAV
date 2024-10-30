#include "tracker/Tracker.h"

Tracker::Tracker()
	: cur_id_{ 0 },
		distance_threshold_{ 2.0f }, // Distance threshold: we set a reasonable value to filter objects far from the traces
		covariance_threshold{ 1.0f },// Covariance threshold: we set a reasonable value for the quality of estimates
		loss_threshold{ 3 },				 // Loss threshold: number of frames in which a trace may not be visible before it is removed
		tracks_{},
		associated_track_det_ids_{}
{ }

/** This function removes tracks based on any strategy */
void Tracker::removeTracks()
{
	static std::vector<Tracklet> tracks_to_keep;
	tracks_to_keep.clear();
	
	for (Tracklet& track : tracks_)
	{
		//Tracklets that have not been updated for a number of consecutive frames are deleted.
		if (track.getLossCount() < loss_threshold) 
			tracks_to_keep.emplace_back(track.getId(), track.getX(), track.getY());
	}

	// Keep only valid traces
	tracks_.swap(tracks_to_keep);
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
