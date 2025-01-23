#include "tracker/Tracker.h"

#include <iostream>

void Tracker::removeTracks()
{
	if (tracks_.empty())
		return;

	// Avoiding Multiple Calls to tracks_.erase using std::remove_if
	auto lambda = [&](Tracklet &track)
	{
		if (track.getLossCount() > loss_threshold_) // The track's loss count exceeds the loss_threshold.
		{
			std::printf("Removing Tracklet ID %d due to excessive loss count (%d > %d)\n",
									track.getId(),
									track.getLossCount(),
									loss_threshold_);
			return true;
		}
		if (track.getXCovariance() > covariance_threshold_ || // The track's X covariance exceeds the covariance_threshold.
				track.getYCovariance() > covariance_threshold_)		// The track's Y covariance exceeds the covariance_threshold.
		{
			std::printf("Removing Tracklet ID %d due to excessive uncertainty (%f, %f) > %f\n",
									track.getId(),
									track.getXCovariance(),
									track.getYCovariance(),
									covariance_threshold_);
			return true;
		}

		return false;
	};
	auto new_end = std::remove_if(tracks_.begin(), tracks_.end(), lambda);
	// Removes marked tracklets
	tracks_.erase(new_end, tracks_.end());
}

void Tracker::addTracks(
		const std::vector<bool> &associated_detections,
		const std::vector<double> &centroids_x,
		const std::vector<double> &centroids_y)
{
	// Iterates through all detections
	for (size_t i = 0; i < associated_detections.size(); i++)
	{
		// if associated_detections[i] is false, it means that the detection is not associated with any active trace.
		if (!associated_detections[i])
		{
			std::printf("Creating new tracklet at position (%f, %f)\n",
									centroids_x[i],
									centroids_y[i]);

			// A new tracklet is created using the coordinates (centroids_x[i], centroids_y[i]).
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
	// Iteration on existing tracks (tracks_)
	for (size_t i = 0; i < tracks_.size(); i++)
	{
		Tracklet &track = tracks_[i];

		// Track uncertainty check:
		// If the covariance of the track exceeds the threshold specified by covariance_threshold_,
		// the track is ignored. This avoids associating detections with traces with high uncertainty.
		if (track.getXCovariance() > covariance_threshold_ ||
				track.getYCovariance() > covariance_threshold_)
		{
			std::printf("Ignore track: the covariance (%f, %f) exceeds the threshold specified (%f)\n",
									track.getXCovariance(),
									track.getYCovariance(),
									covariance_threshold_);

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
			double distance = std::sqrt(pow(dx, 2) + pow(dy, 2));
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
#endif
}

void Tracker::track(
		const std::vector<double> &centroids_x,
		const std::vector<double> &centroids_y,
		bool lidarStatus)
{
	std::vector<bool> associated_detections(centroids_x.size(), false);

	// 1. Prediction
	// For each existing tracklet, the system predicts its future position using the Kalman filter.
	for (Tracklet &track : tracks_)
	{
		std::printf("Tracklet ID %d pre-prediction state(%f, %f)\n",
								track.getId(),
								track.getX(),
								track.getY());

		track.predict();

		std::printf("Tracklet ID %d post-prediction state(%f, %f)\n",
								track.getId(),
								track.getX(),
								track.getY());
	}

	// 2. Data Association
	// After prediction, current detections (centroids) are associated with existing tracks,
	// using a distance criterion (Euclidean distance)
	dataAssociation(associated_detections, centroids_x, centroids_y);

	// 3. Update
	// Once detections are associated with tracks, the track data is updated with the new information
	// from the detections. If a track is associated with a detection, its location is updated with the
	// location of the detection.
	for (size_t i = 0; i < tracks_.size(); ++i)
	{
		if (associated_detections[i])
		{
			tracks_[i].update(centroids_x[i], centroids_y[i], lidarStatus);
		}
	}

	// 4. Add new tracks
	// If there are detections that have not been associated with existing tracks,
	// new tracks (tracklets) are created for these detections.
	addTracks(associated_detections, centroids_x, centroids_y);

	// 5. Remove tracks
	// Tracks that have not been associated with detections for a number of frames,
	// or that have too high an uncertainty (e.g., due to high covariance), are removed.
	removeTracks();
}
