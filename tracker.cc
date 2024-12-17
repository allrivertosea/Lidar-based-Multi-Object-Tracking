
#include "tracker.h"

#include <ctime>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <string>

#include "visualizer.h"

namespace {

static constexpr double kTrackLidarDetectionDistanceGating = 3.0;// 距离门限，表示在数据关联时，激光雷达检测与跟踪目标之间的最大允许距离

// 计算激光雷达检测与当前目标轨迹之间的距离
double GetTrackLidarDetectionDistance(const LidarDetection& detection,
                                      const Track& track) {
  const Vec3d error = detection.position() - track.position();
  return error.Length();// 返回两者之间的欧几里得距离
}

}  // namespace

// 跟踪器的运行函数，处理每一帧数据
void Tracker::Run(const LidarFrame& frame) {
  std::cout << "\nLidar frame_index: " << frame.index()
            << " detections_num: " << frame.detections().size()
            << " timestamp: " << std::fixed << std::setw(16)
            << std::setprecision(6) << frame.timestamp() << std::endl;

  // 预测当前跟踪目标的状态
  PredictTracks(frame.timestamp());

  std::vector<std::pair<int, int>> association_pairs;// 用于存储关联结果
  std::vector<int> unassociated_track_indices;       // 用于存储未关联的跟踪目标索引
  std::vector<int> unassociated_detection_indices;   // 用于存储未关联的检测结果索引
  DataAssociation(frame, &association_pairs, &unassociated_track_indices,
                  &unassociated_detection_indices);  // 进行数据关联
  // 更新已经关联的跟踪目标
  UpdateTracks(frame, association_pairs);
  // 处理未关联的目标，包括管理新目标和移除丢失的目标
  ManagementTracks(frame, unassociated_track_indices,
                   unassociated_detection_indices);
  // 可视化结果
  Visualizer(frame, tracks_);
}

// 预测当前跟踪目标的状态
void Tracker::PredictTracks(const double& timestamp) {
  for (Track& track : tracks_) {
    track.Predict(timestamp);// 更新每个跟踪目标的预测状态
  }
}

// 数据关联，关联激光雷达检测与现有的跟踪目标
void Tracker::DataAssociation(
    const LidarFrame& frame,
    std::vector<std::pair<int, int>>* association_pairs,
    std::vector<int>* unassociated_track_indices,
    std::vector<int>* unassociated_detection_indices) {

  // 检查指针是否合法
  if (association_pairs == nullptr || unassociated_track_indices == nullptr ||
      unassociated_detection_indices == nullptr) {
    return;
  }
  // 如果当前帧没有任何检测结果，直接返回
  if (frame.detections().empty()) {
    std::cout << "frame is empty! index: " << frame.index()
              << " timestamp: " << std::fixed << std::setw(16)
              << std::setprecision(6) << frame.timestamp() << std::endl;
    return;
  }
  // 如果当前没有任何跟踪目标，则将所有检测结果作为未关联的目标
  if (tracks_.empty()) {
    std::cout << "tracks is empty! frame index: " << frame.index()
              << " timestamp: " << std::fixed << std::setw(16)
              << std::setprecision(6) << frame.timestamp() << std::endl;
    unassociated_detection_indices->resize(frame.detections().size());
    std::iota(unassociated_detection_indices->begin(),
              unassociated_detection_indices->end(), 0);
    return;
  }

  const std::vector<LidarDetection>& detections = frame.detections();// 获取当前帧的所有检测结果
  std::vector<bool> is_track_associated(tracks_.size(), false);// 用于记录每个跟踪目标是否已被关联
  //遍历每个检测，看和哪个跟踪可以关联上
  for (int i = 0; i < detections.size(); ++i) {
    double min_distance = kTrackLidarDetectionDistanceGating;// 初始化最小距离
    int nearest_track_index = -1;// 用于存储最近的跟踪目标索引
    for (int j = 0; j < tracks_.size(); ++j) {
      if (is_track_associated[j]) continue;// 如果该跟踪目标已经关联，则跳过
      const double distance = GetTrackLidarDetectionDistance(detections[i], tracks_[j]);// 计算当前检测与每个跟踪目标的距离
      if (distance < min_distance) {// 如果该目标更接近，更新最小距离和目标索引
        min_distance = distance;
        nearest_track_index = j;
      }
    }
    // 如果有跟踪目标被检测关联上，将其添加到关联对中(跟踪索引，检测索引)，否则将该检测索引放入未关联检测列表中
    if (nearest_track_index > 0) {
      association_pairs->emplace_back(nearest_track_index, i);
      is_track_associated[nearest_track_index] = true;
    } else {
      unassociated_detection_indices->push_back(i);
    }
  }
  // 将未关联的跟踪目标添加到未关联跟踪目标列表中
  for (int i = 0; i < is_track_associated.size(); ++i) {
    if (!is_track_associated[i]) {
      unassociated_track_indices->push_back(i);
    }
  }
}
// 根据关联对，使用对应观测来更新已关联的跟踪目标
void Tracker::UpdateTracks(
    const LidarFrame& frame,
    const std::vector<std::pair<int, int>>& association_pairs) {
  const std::vector<LidarDetection>& detections = frame.detections();// 获取当前帧的所有检测结果
  for (const auto& pair : association_pairs) {
    const int track_index = pair.first;
    const int detection_index = pair.second;
    tracks_[track_index].Update(detections[detection_index]);// 更新每个关联的跟踪目标
  }
}

// 管理跟踪目标，包括创建新目标和移除丢失目标
void Tracker::ManagementTracks(
    const LidarFrame& frame, const std::vector<int>& unassociated_track_indices,
    const std::vector<int>& unassociated_detection_indices) {
  // 对未关联的检测结果创建新的跟踪目标
  const std::vector<LidarDetection>& detections = frame.detections();
  for (const int index : unassociated_detection_indices) {
    tracks_.emplace_back(detections[index]);
  }

  // 移除丢失的跟踪目标
  auto iter = tracks_.begin();
  while (iter != tracks_.end()) {
    if (iter->IsLost()) {
      iter = tracks_.erase(iter);
    } else {
      ++iter;
    }
  }
}

// 发布当前帧的跟踪结果，并写入文件
std::vector<Track> Tracker::PublishTracks(const LidarFrame& frame) {
  std::vector<Track> published_tracks;
  for (const Track& track : tracks_) {
    if (track.IsConfirmed()) {// 如果跟踪目标已确认，添加到结果中
      published_tracks.push_back(track);
    }
  }
  // 打开输出文件以写入结果
  std::ofstream output_file(output_filename_, std::ios::out | std::ios::app);

  if (!output_file.is_open()) {
    std::cout << "Fail to open " << output_filename_ << std::endl;
    return published_tracks;
  }
  // 写入跟踪结果
  output_file << frame.index() << " " << published_tracks.size() << " "
              << std::fixed << std::setw(16) << std::setprecision(6)
              << frame.timestamp() << std::endl;
  // 写入每个已确认的跟踪目标的详细信息
  if (!published_tracks.empty()) {
    for (int i = 0; i < tracks_.size(); ++i) {
      if (tracks_[i].IsConfirmed()) {
        output_file << tracks_[i].id() << "  " << tracks_[i].position().x()
                    << " " << tracks_[i].position().y() << " "
                    << tracks_[i].position().z() << " "
                    << tracks_[i].velocity().x() << " "
                    << tracks_[i].velocity().y() << " "
                    << tracks_[i].velocity().z() << " "
                    << tracks_[i].acceleration().x() << " "
                    << tracks_[i].acceleration().y() << " "
                    << tracks_[i].acceleration().z() << " "
                    << tracks_[i].size().x() << " " << tracks_[i].size().y()
                    << " " << tracks_[i].size().z() << " " << tracks_[i].yaw()
                    << " " << tracks_[i].yaw_rate() << std::endl;
      }
    }
  }

  output_file << std::endl;
  output_file.close();// 关闭文件
  std::cout << "Publish " << published_tracks.size() << " tracks to "
            << output_filename_ << " at timestamp: " << std::fixed
            << std::setw(16) << std::setprecision(6) << frame.timestamp()
            << std::endl;
  return published_tracks;
}