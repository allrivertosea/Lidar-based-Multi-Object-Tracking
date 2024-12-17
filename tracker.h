#pragma once

#include <vector>

#include "lidar_detection.h"
#include "track.h"

class Tracker {
 public:
  Tracker() = default;
  ~Tracker() = default;

  Tracker(const std::string& output_filename)
      : output_filename_(output_filename) {}

  void Run(const LidarFrame& frame);

  void PredictTracks(const double& timestamp);

  void DataAssociation(const LidarFrame& frame,
                       std::vector<std::pair<int, int>>* association_pairs,
                       std::vector<int>* unassociated_track_indices,
                       std::vector<int>* unassociated_detection_indices);

  void UpdateTracks(const LidarFrame& frame,
                    const std::vector<std::pair<int, int>>& association_pairs);

  void ManagementTracks(const LidarFrame& frame,
                        const std::vector<int>& unassociated_track_indices,
                        const std::vector<int>& unassociated_detection_indices);

  std::vector<Track> PublishTracks(const LidarFrame& frame);

 private:
  std::vector<Track> tracks_;
  const std::string output_filename_;
};
