#include "tracker/Tracker.h"

#include <iostream>
#include <thread>
#include <set>

static double calculateEuclideanDistance(double x1,
																				 double y1,
																				 double x2,
																				 double y2)
{
	double dx = x1 - x2;
	double dy = y1 - y2;
	return std::sqrt(pow(dx, 2) + pow(dy, 2));
}

void Tracker::removeTracks()
{
	if (tracks_.empty())
		return;

	// Avoiding Multiple Calls to tracks_.erase using std::remove_if
	auto lambda = [&](Tracklet &track)
	{
		if (track.getLossCount() > loss_threshold_) // The track's loss count exceeds the loss_threshold.
		{
			std::printf(
					"Removing Tracklet %d due to excessive loss count (%d > %d)\n",
					track.getId(),
					track.getLossCount(),
					loss_threshold_);
			return true;
		}
		if (track.getXCovariance() > covariance_threshold_ || // The track's X covariance exceeds the covariance_threshold.
				track.getYCovariance() > covariance_threshold_)		// The track's Y covariance exceeds the covariance_threshold.
		{
			std::printf(
					"Removing Tracklet %d due to excessive uncertainty (%f, %f) > %f\n",
					track.getId(),
					track.getXCovariance(),
					track.getYCovariance(),
					covariance_threshold_);

			return true;
		}

		return false;
	};
	auto new_end = std::remove_if(tracks_.begin(), tracks_.end(), lambda);
	tracks_.erase(new_end, tracks_.end());
}

void Tracker::addTracks(const std::vector<bool> &associated_detections,
												const std::vector<double> &centroids_x,
												const std::vector<double> &centroids_y)
{
	// Iterates through all detections
	for (size_t i = 0; i < associated_detections.size(); i++)
	{
		// if associated_detections[i] is false, it means that the detection is not associated with any active trace.
		if (!associated_detections.at(i))
		{
			// A new tracklet is created using the coordinates (centroids_x[i], centroids_y[i]).
			tracks_.emplace_back(
					cur_id_,
					centroids_x.at(i),
					centroids_y.at(i));

			std::printf(
					"New tracklet %d created at position (%f, %f)\n",
					cur_id_,
					centroids_x.at(i),
					centroids_y.at(i));

			cur_id_++;
		}
	}
}

void Tracker::dataAssociation(std::vector<bool> &associated_detections,
															const std::vector<double> &centroids_x,
															const std::vector<double> &centroids_y)
{
	// Iterate on all detections
	for (size_t i = 0; i < associated_detections.size(); i++)
	{
		auto current_detection = associated_detections.at(i);

		// Skip detection if it is already associated
		if (current_detection)
			continue;

		double min_distance = std::numeric_limits<double>::max();
		int best_track_id = -1;

		// Iterate on all tracklets to find the nearest
		for (size_t j = 0; j < tracks_.size(); ++j)
		{
			auto &track = tracks_.at(j);

			// Track uncertainty check:
			// If the covariance of the track exceeds the threshold specified by covariance_threshold_,
			// the track is ignored. This avoids associating detections with traces with high uncertainty.
			if (track.getXCovariance() > covariance_threshold_ ||
					track.getYCovariance() > covariance_threshold_)
			{
				std::printf(
						"Ignore track %d: covariance (%f, %f) exceeds the threshold specified (%f)\n",
						track.getId(),
						track.getXCovariance(),
						track.getYCovariance(),
						covariance_threshold_);
				continue;
			}

			// Calculate Euclidean distance
			double distance = calculateEuclideanDistance(
					centroids_x.at(i),
					centroids_y.at(i),
					track.getX(),
					track.getY());

			// Update the nearest tracklet
			if (distance < min_distance)
			{
				min_distance = distance;
				best_track_id = j;
			}
		}

		// Associates tracking with the best tracklet, if the distance is acceptable
		if (best_track_id != -1 && min_distance < distance_threshold_)
		{
			auto &best_track = tracks_.at(best_track_id);
			associated_track_det_ids_.emplace_back(best_track.getId(), i);
			current_detection = true;

			// std::printf(
			// 		"Associated detection (%f, %f) to Tracklet %d with distance %f\n",
			// 		centroids_x.at(i),
			// 		centroids_y.at(i),
			// 		tracks_.at(best_track_id).getId(),
			// 		min_distance);
		}
	}
}

void Tracker::track(const std::vector<double> &centroids_x,
										const std::vector<double> &centroids_y,
										bool lidarStatus)
{
	associated_track_det_ids_.clear();

	// std::printf("\n========================================\n");
	// for (int i = 0; i < centroids_x.size(); i++)
	// 	std::printf("Centroid (%f, %f)\n", centroids_x.at(i), centroids_y.at(i));

	// 1. Prediction
	for (auto &track : tracks_)
	{
		double old_x = track.getX();
		double old_y = track.getY();
		track.predict();

		// std::printf(
		// 		"1. Tracklet %d prediction from (%f, %f) to (%f, %f)\n",
		// 		track.getId(),
		// 		old_x,
		// 		old_y,
		// 		track.getX(),
		// 		track.getY());
	}

	// 2. Data Association
	// std::printf("2. Start associated_detections...\n");
	std::vector<bool> associated_detections(centroids_x.size(), false);
	dataAssociation(associated_detections, centroids_x, centroids_y);

	// 3. Update
	for (auto [track_id, detection_id] : associated_track_det_ids_)
	{
		auto lambda = [&](Tracklet &track)
		{ return track.getId() == track_id; };

		auto it = std::find_if(tracks_.begin(), tracks_.end(), lambda);
		if (it != tracks_.end())
		{
			// std::printf("3. Update tracklet %d\n", it->getId());
			it->update(
					centroids_x.at(detection_id),
					centroids_y.at(detection_id),
					lidarStatus);
		}
	}

	// 4. Add new tracks
	// std::printf("4. Add new tracks...\n");
	addTracks(associated_detections, centroids_x, centroids_y);

	// 5. Remove tracks
	// std::printf("5. Remove tracks...\n");
	removeTracks();
	// std::printf("(tracks_.size()=%lu)\n", tracks_.size());
}
