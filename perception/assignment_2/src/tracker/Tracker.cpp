#include "tracker/Tracker.h"

#include <iostream>
#include <thread>

static double calculateEuclideanDistance(double x1, double y1, double x2, double y2)
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
			std::printf("Removing Tracklet %d due to excessive loss count (%d > %d)\n",
									track.getId(),
									track.getLossCount(),
									loss_threshold_);
			return true;
		}
		if (track.getXCovariance() > covariance_threshold_ || // The track's X covariance exceeds the covariance_threshold.
				track.getYCovariance() > covariance_threshold_)		// The track's Y covariance exceeds the covariance_threshold.
		{
			std::printf("Removing Tracklet %d due to excessive uncertainty (%f, %f) > %f\n",
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
		if (!associated_detections.at(i))
		{
			std::printf("Creating new tracklet %d at position (%f, %f)\n",
									cur_id_,
									centroids_x.at(i),
									centroids_y.at(i));

			// A new tracklet is created using the coordinates (centroids_x[i], centroids_y[i]).
			tracks_.emplace_back(
					cur_id_,
					centroids_x.at(i),
					centroids_y.at(i));

			cur_id_++;
		}
	}
}

void Tracker::dataAssociation(
		std::vector<bool> &associated_detections,
		const std::vector<double> &centroids_x,
		const std::vector<double> &centroids_y)
{
	// Itera su tutti i rilevamenti
	for (size_t i = 0; i < associated_detections.size(); i++)
	{
		auto current_detection = associated_detections.at(i);

		// Salta il rilevamento se è già associato
		if (current_detection)
			continue;

		double min_distance = std::numeric_limits<double>::max();
		int best_track_id = -1;

		// Itera su tutti i tracklet per trovare il più vicino
		for (size_t j = 0; j < tracks_.size(); ++j)
		{
			auto &track = tracks_.at(j);

			// Track uncertainty check:
			// If the covariance of the track exceeds the threshold specified by covariance_threshold_,
			// the track is ignored. This avoids associating detections with traces with high uncertainty.
			if (track.getXCovariance() > covariance_threshold_ ||
					track.getYCovariance() > covariance_threshold_)
			{
				std::printf("Ignore track %d: covariance (%f, %f) exceeds the threshold specified (%f)\n",
										track.getId(),
										track.getXCovariance(),
										track.getYCovariance(),
										covariance_threshold_);

				continue;
			}

			// Calcola la distanza Euclidea
			double distance = calculateEuclideanDistance(
					centroids_x.at(i),
					centroids_y.at(i),
					track.getX(),
					track.getY());

			// Aggiorna il tracklet più vicino
			if (distance < min_distance)
			{
				min_distance = distance;
				best_track_id = j;
			}
		}

		// Associa il rilevamento al tracklet migliore, se la distanza è accettabile
		if (best_track_id != -1 && min_distance < distance_threshold_)
		{
			auto &best_track = tracks_.at(best_track_id);
			associated_track_det_ids_.emplace_back(best_track.getId(), i);
			current_detection = true;

			// Log di debug per monitorare l'associazione
			std::printf("Associated detection (%f, %f) to Tracklet %d with distance %f\n",
									centroids_x.at(i),
									centroids_y.at(i),
									tracks_.at(best_track_id).getId(),
									min_distance);
		}
	}
}

void Tracker::track(
		const std::vector<double> &centroids_x,
		const std::vector<double> &centroids_y,
		bool lidarStatus)
{
	associated_track_det_ids_.clear();

	// std::printf("\n========================================\n");
	// for (int i = 0; i < centroids_x.size(); i++)
	// 	std::printf("Centroid (%f, %f)\n", centroids_x.at(i), centroids_y.at(i));
	// std::printf("1. Start predict...\n");

	// 1. Prediction
	// For each existing tracklet, the system predicts its future position using the Kalman filter.
	for (auto &track : tracks_)
	{
		double old_x = track.getX();
		double old_y = track.getY();
		track.predict();

		std::printf("Tracklet %d prediction from (%f, %f) to (%f, %f)\n",
								track.getId(),
								old_x,
								old_y,
								track.getX(),
								track.getY());
	}

	// std::this_thread::sleep_for(std::chrono::seconds(1));
	// std::printf("2. Start associated_detections...\n");

	// 2. Data Association
	// After prediction, current detections (centroids) are associated with existing tracks,
	// using a distance criterion (Euclidean distance)
	std::vector<bool> associated_detections(centroids_x.size(), false);
	dataAssociation(associated_detections, centroids_x, centroids_y);

	// std::this_thread::sleep_for(std::chrono::seconds(1));
	// std::printf("3. Start update...\n");

	// 3. Update
	// Once detections are associated with tracks, the track data is updated with the new information
	// from the detections. If a track is associated with a detection, its location is updated with the
	// location of the detection.
	for (auto [track_id, detection_id] : associated_track_det_ids_)
	{
		std::printf("Update track %d\n", track_id);
		tracks_.at(track_id).update(
				centroids_x.at(detection_id),
				centroids_y.at(detection_id),
				lidarStatus);
	}

	// std::this_thread::sleep_for(std::chrono::seconds(1));
	// std::printf("4. Add new tracks...\n");

	// 4. Add new tracks
	// If there are detections that have not been associated with existing tracks,
	// new tracks (tracklets) are created for these detections.
	addTracks(associated_detections, centroids_x, centroids_y);

	// std::this_thread::sleep_for(std::chrono::seconds(1));
	// std::printf("5. Remove tracks...\n");

	// 5. Remove tracks
	// Tracks that have not been associated with detections for a number of frames,
	// or that have too high an uncertainty (e.g., due to high covariance), are removed.
	removeTracks();

	// std::this_thread::sleep_for(std::chrono::seconds(1));
}
