#pragma once

#include <iostream>
#include <vector>

#include "math.h"

class LidarDetection {
 public:
  LidarDetection() = default;
  ~LidarDetection() = default;

  LidarDetection(const uint32_t id, const double timestamp, Vec3d position,
                 Vec3d size, double yaw)
      : id_(id),
        timestamp_(timestamp),
        position_(position),
        size_(size),
        yaw_(yaw),
        distance_to_ego_(-1.0) {}

  uint32_t id() const { return id_; }
  double timestamp() const { return timestamp_; }
  Vec3d position() const { return position_; }
  Vec3d size() const { return size_; }
  double yaw() const { return yaw_; }
  double distance_to_ego() const { return distance_to_ego_; }

  void GetDistanceToEgo(const Vec2d ego_position) {
    const Vec2d vector_to_ego = position_.xy() - ego_position;
    distance_to_ego_ = vector_to_ego.Length();
  }

 private:
  uint32_t id_;
  double timestamp_;
  Vec3d position_;
  Vec3d size_;  // Bounding box size in meters as width, length, height.
  double yaw_;
  double distance_to_ego_;
};

class LidarFrame {
 public:
  LidarFrame() = default;
  ~LidarFrame() = default;

  LidarFrame(uint32_t index, double timestamp,
             const Transformation2d& world_to_vehicle,
             const std::vector<LidarDetection>& detections)
      : index_(index),
        timestamp_(timestamp),
        world_to_vehicle_(world_to_vehicle),
        detections_(detections) {}

  uint32_t index() const { return index_; }
  double timestamp() const { return timestamp_; }
  const Transformation2d& world_to_vehicle() const { return world_to_vehicle_; }
  const std::vector<LidarDetection>& detections() const { return detections_; }

 private:
  uint32_t index_;
  double timestamp_;
  Transformation2d world_to_vehicle_;
  std::vector<LidarDetection> detections_;
};
