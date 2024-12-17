#pragma once

#include <vector>

#include "hungarian.h"
#include "track.h"

namespace {

static constexpr double kMaximumDistanceGating = 2.0;// 最大距离门限，用于数据关联时的距离判断

// 计算两条轨迹之间的距离
double CalculateDistance(const Track& t1, const Track& t2) {
  const Vec3d distance_vector = t1.position() - t2.position();// 计算位置差值向量
  return distance_vector.Length();
}
// 获取距离矩阵，用于计算每一对轨迹之间的距离
std::vector<std::vector<double>> GetDistanceMatrix(
    const std::vector<Track>& gt_tracks,
    const std::vector<Track>& published_tracks) {
  std::vector<std::vector<double>> distances;
  // 对每一对真实轨迹和发布轨迹计算距离
  for (const Track& gt_track : gt_tracks) {
    std::vector<double> distance;
    for (const Track& pub_track : published_tracks) {
      distance.push_back(CalculateDistance(gt_track, pub_track));
    }
    distances.push_back(distance);
  }
  return distances;
}

// 继承上一次的关联结果并更新当前的轨迹对应关系
std::vector<std::pair<uint32_t, uint32_t>> InheritCorrespondence(
    const std::vector<std::pair<uint32_t, uint32_t>>& last_correspondences,
    std::vector<Track>* gt_tracks, std::vector<Track>* published_tracks) {
  std::vector<std::pair<uint32_t, uint32_t>> correspondences;
  // 遍历上一次的关联结果
  for (const auto& pair : last_correspondences) {
    const uint32_t& last_gt_track_id = pair.first;
    const uint32_t& last_pub_track_id = pair.second;
    // 查找当前帧中的真实轨迹和发布轨迹
    auto current_gt_track_iter =
        std::find_if(gt_tracks->begin(), gt_tracks->end(),
                     [&last_gt_track_id](const Track& track) -> bool {
                       return track.id() == last_gt_track_id;
                     });
    auto current_pub_track_iter =
        std::find_if(published_tracks->begin(), published_tracks->end(),
                     [&last_pub_track_id](const Track& track) -> bool {
                       return track.id() == last_pub_track_id;
                     });
    // 如果找到了匹配的轨迹并且它们之间的距离小于最大距离门限，则认为它们仍然匹配
    if (current_gt_track_iter == gt_tracks->end() ||
        current_pub_track_iter == published_tracks->end()) {
      continue;
    }
    if (CalculateDistance(*current_gt_track_iter, *current_pub_track_iter) <
        kMaximumDistanceGating) {
      correspondences.emplace_back(last_gt_track_id, last_pub_track_id);
      gt_tracks->erase(current_gt_track_iter);// 移除已经匹配的真实轨迹
      published_tracks->erase(current_pub_track_iter);// 移除已经匹配的发布轨迹
    }
  }
  return correspondences;
}

}  // namespace
// 性能评估函数，计算 MOTA（多目标跟踪准确度）
double PerformanceEvaluation(
    const std::vector<std::vector<Track>>& published_tracks_list,
    const std::vector<std::vector<Track>>& gt_tracks_list) {
  if (published_tracks_list.size() != gt_tracks_list.size()) {
    std::cout << "Frames number is wrong! published_tracks_list: "
              << published_tracks_list.size()
              << " gt_tracks_list: " << gt_tracks_list.size() << std::endl;
    return 0.0;// 如果帧数不匹配，返回 0
  }

  std::cout << "\nPerformance Evaluation Results" << std::endl;

  int ground_truth_num = 0;// 统计真实轨迹数量
  int miss_detection_num = 0;// 统计漏检数量
  int false_positive_num = 0;// 统计误检数量
  int mismatch_num = 0;// 统计误匹配数量

  const int frame_num = gt_tracks_list.size(); // 总帧数
  std::vector<std::pair<uint32_t, uint32_t>> last_correspondences;// 存储上一次的轨迹对应关系
  // 遍历每一帧
  for (int k = 0; k < frame_num; ++k) {
    ground_truth_num += gt_tracks_list[k].size();// 累加真实轨迹数量
    std::vector<Track> gt_tracks = gt_tracks_list[k];
    std::vector<Track> pub_tracks = published_tracks_list[k];
    // 继承上一次的轨迹对应关系
    std::vector<std::pair<uint32_t, uint32_t>> current_correspondences =
        InheritCorrespondence(last_correspondences, &gt_tracks, &pub_tracks);

    if (!gt_tracks.empty() && !pub_tracks.empty()) {
      // 计算当前帧中真实轨迹与发布轨迹之间的距离矩阵，用于计算新的对应关系
      const std::vector<std::vector<double>> distances =
          GetDistanceMatrix(gt_tracks, pub_tracks);
      std::vector<std::pair<uint32_t, uint32_t>> associations;
      std::vector<uint32_t> unassociated_rows;
      std::vector<uint32_t> unassociated_cols;
      // 使用匈牙利算法进行数据关联
      HungarianMinimize(distances, kMaximumDistanceGating, &associations,
                        &unassociated_rows, &unassociated_cols);

      int mismatch_cnt = 0;// 误匹配计数
      int miss_detections_cnt = unassociated_rows.size();// 漏检计数
      int false_positives_cnt = unassociated_cols.size();// 误检计数
      int pair_cnt = 0;// 匹配对计数
      for (const auto& pair : associations) {
        if (distances[pair.first][pair.second] < kMaximumDistanceGating) {// 如果匹配的距离小于最大距离门限，认为是有效匹配
          const uint32_t gt_track_id = gt_tracks[pair.first].id();
          const uint32_t pub_track_id = pub_tracks[pair.second].id();
          current_correspondences.emplace_back(gt_track_id, pub_track_id);
          auto last_correspondence_iter = std::find_if(// 检查是否发生了误匹配
              last_correspondences.begin(), last_correspondences.end(),
              [&gt_track_id](const std::pair<uint32_t, uint32_t>& id_pair)
                  -> bool { return id_pair.first == gt_track_id; });
          // 如果之前匹配的真实轨迹对应的发布轨迹不同，认为是误匹配
          if (last_correspondence_iter != last_correspondences.end() &&
              last_correspondence_iter->second != pub_track_id) {
            ++mismatch_cnt;
          }
        } else {
          ++miss_detections_cnt;// 如果距离大于门限，认为是漏检
          ++false_positives_cnt;//认为是误检测
        }
      }

      miss_detection_num += miss_detections_cnt;// 更新漏检数
      false_positive_num += false_positives_cnt;// 更新误检数
      mismatch_num += mismatch_cnt;// 更新误匹配数
    } else if (gt_tracks.empty() && !pub_tracks.empty()) {
      false_positive_num += pub_tracks.size();// 如果当前帧只有发布轨迹，没有真实轨迹，认为所有发布轨迹都是误检
    } else if (pub_tracks.empty() && !gt_tracks.empty()) {
      miss_detection_num += gt_tracks.size();//如果当前帧只有真实轨迹，没有发布轨迹，认为所有真实轨迹都是漏检
    }

    last_correspondences = current_correspondences; // 更新上一次的轨迹对应关系
  }
  // 计算漏检率、虚假正例率和误匹配率
  const double miss_detection_rate = static_cast<double>(miss_detection_num) /
                                     static_cast<double>(ground_truth_num);
  const double false_positive_rate = static_cast<double>(false_positive_num) /
                                     static_cast<double>(ground_truth_num);
  const double mismatch_rate =
      static_cast<double>(mismatch_num) / static_cast<double>(ground_truth_num);
  // 计算 MOTA（多目标跟踪准确度）
  const double mota =
      1 - miss_detection_rate - false_positive_rate - mismatch_rate;
  // 输出评估结果
  std::cout << "miss_detection_num: " << miss_detection_num
            << " false_positive_num: " << false_positive_num
            << " mismatch_num: " << mismatch_num
            << " ground_truth_num: " << ground_truth_num << std::endl;
  std::cout << "miss_detection_rate: " << miss_detection_rate
            << " false_positive_rate: " << false_positive_rate
            << " mismatch_rate: " << mismatch_rate << "  MOTA: " << mota
            << std::endl;

  return mota;
}
