#include "tracker/Tracker.h"

#include <iostream>

void Tracker::removeTracks()
{
	if (tracks_.empty())
		return;

	// We invert the tracks_ vector to avoid iterator invalidation problems when we delete elements.
	// for (size_t i = tracks_.size(); i > 0; i--)
	// {
	// 	auto &track = tracks_[i - 1];
	// 	if (track.getLossCount() > loss_threshold ||					// The track's loss count exceeds the loss_threshold.
	// 			track.getXCovariance() > covariance_threshold ||	// The track's X covariance exceeds the covariance_threshold.
	// 			track.getYCovariance() > covariance_threshold)		// The track's Y covariance exceeds the covariance_threshold.
	// 	{
	// 		std::cout << "Removing Tracklet ID " << track.getId()
	// 							<< " due to excessive loss count or uncertainty."
	// 							<< std::endl;
	// 		tracks_.erase(tracks_.begin() + (i - 1));
	// 	}
	// }

	// Avoiding Multiple Calls to tracks_.erase using std::remove_if
	auto lambda = [&](Tracklet &track)
	{
		if (track.getLossCount() > loss_threshold ||				 // The track's loss count exceeds the loss_threshold.
				track.getXCovariance() > covariance_threshold || // The track's X covariance exceeds the covariance_threshold.
				track.getYCovariance() > covariance_threshold)	 // The track's Y covariance exceeds the covariance_threshold.
		{
			std::cout << "Removing Tracklet ID " << track.getId()
								<< " due to excessive loss count or uncertainty."
								<< std::endl;
			return true;
		}
		return false;
	};
	auto new_end = std::remove_if(tracks_.begin(), tracks_.end(), lambda);
	tracks_.erase(new_end, tracks_.end());
}

void Tracker::addTracks(
		const std::vector<bool> &associated_detections,
		const std::vector<double> &centroids_x,
		const std::vector<double> &centroids_y)
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

			// Tracklet(int idTrack, double x, double y);
			tracks_.emplace_back(cur_id_++, centroids_x[i], centroids_y[i]);
		}
	}
}

void Tracker::dataAssociation(
		std::vector<bool> &associated_detections,
		const std::vector<double> &centroids_x,
		const std::vector<double> &centroids_y)
{
#if 0
	// Remind this vector contains a pair of tracks and its corresponding
	associated_track_det_ids_.clear();

	for (size_t i = 0; i < tracks_.size(); ++i)
	{
		int closest_point_id = -1;
		double min_dist = std::numeric_limits<double>::max();

		for (size_t j = 0; j < associated_detections.size(); ++j)
		{
			// TODO
			// Implement logic to find the closest detection (centroids_x,centroids_y)
			// to the current track (tracks_)
		}

		// Associate the closest detection to a tracklet
		if (min_dist < distance_threshold_ && !associated_detections[closest_point_id])
		{
			associated_track_det_ids_.push_back(std::make_pair(closest_point_id, i));
			associated_detections[closest_point_id] = true;
		}
	}
#endif
}

void Tracker::track(
		const std::vector<double> &centroids_x,
		const std::vector<double> &centroids_y,
		bool lidarStatus)
{
	std::cout << "Starting tracking cycle..." << std::endl;

	// ======================================
	// 1. Prediction
	// ======================================
	//
	// Before associating new detections, each existing tracklet calls the predict method
	// to update the predicted state.
	// This helps estimate where the tracklets will be at the next round of detections.
	for (Tracklet &track : tracks_)
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
	associated_detections.assign(centroids_x.size(), false);
	associated_track_det_ids_.clear();

	// dataAssociation(associated_detections, centroids_x, centroids_y);
	// for (size_t i = 0; i < tracks_.size(); i++)
	// {
	// 	std::cout << "Tracklet ID " << tracks_[i].getId() << " is associated with detection: "
	// 						<< (associated_detections[i] ? "Yes" : "No")
	// 						<< std::endl;
	// }

	// ======================================
	// 3. Addition of New Tracklets
	// ======================================
	//
	// Use the addTracks method to create new tracklets for detections that have not been associated
	// with any existing tracklet.
	// Each new unassociated detection generates a new tracklet with a unique ID.
	addTracks(associated_detections, centroids_x, centroids_y);

	// ======================================
	// 4. Remove tracklets
	// ======================================
	// Call the removeTracks method to remove tracklets that have too many update failures
	// (exceed loss_threshold) or too much uncertainty (exceed covariance_threshold)
	removeTracks();

	std::cout << "Tracking cycle completed." << std::endl;
}
