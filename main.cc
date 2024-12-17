#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "evaluation.h"
#include "hungarian.h"
#include "lidar_detection.h"
#include "tracker.h"

namespace {

// 记录输出文件的时间戳，用于生成唯一的输出文件名
static uint64_t output_filename_timestamp = 0;

// 获取输出文件名
std::string GetOutputFilename(const std::string& scene_name) {
  if (output_filename_timestamp == 0) {// 如果时间戳还没有初始化，则获取当前时间
    time_t nowtime;
    time(&nowtime);
    output_filename_timestamp = static_cast<uint64_t>(nowtime);
  }
  // 根据场景名和时间戳生成输出文件名
  const std::string output_filename =
      "../results/" + scene_name + "_results_" +
      std::to_string(output_filename_timestamp) + ".dat";
  return output_filename;
}

}  // namespace

// 从文件中获取地面真值轨迹数据，是世界坐标系下的值
std::vector<std::vector<Track>> GetGroundTruthTracksFromFile(
    const std::string& scene_name) {
   // 构造地面真值轨迹文件的路径
  const std::string gt_tracks_filename =
      "../data/" + scene_name + "_gt_tracks.dat";
  std::ifstream srcFile(gt_tracks_filename, std::ios::in);
  if (!srcFile.is_open()) {
    std::cout << "Fail to open " << gt_tracks_filename << std::endl;
    return std::vector<std::vector<Track>>();
  }

  int frames_num;
  srcFile >> frames_num;// 从文件中读取帧数
  std::vector<std::vector<Track>> tracks_list;// 存储每一帧的轨迹数据
  tracks_list.reserve(frames_num);

  // 遍历每一帧的数据
  for (int i = 0; i < frames_num; ++i) {
    uint32_t frame_index = 0, tracks_num = 0;
    double timestamp = 0;
    srcFile >> frame_index >> tracks_num >> timestamp;// 读取一帧的基本信息

    std::vector<Track> tracks;// 当前帧的轨迹数据
    tracks.reserve(tracks_num);
    // 遍历当前帧中的所有轨迹
    for (int j = 0; j < tracks_num; ++j) {
      uint32_t id;// 轨迹ID
      double x, y, z, l, w, h, yaw;// 轨迹的位置信息和尺寸信息
      srcFile >> id >> x >> y >> z >> l >> w >> h >> yaw;// 读取轨迹的详细信息
      // 创建LidarDetection对象，并添加到当前帧的轨迹列表中
      tracks.emplace_back(
          LidarDetection(id, timestamp, Vec3d(x, y, z), Vec3d(l, w, h), yaw));
      tracks.back().SetId(id);// 设置轨迹ID
    }
    tracks_list.push_back(tracks);// 将当前帧的轨迹列表添加到总轨迹列表中
  }
  srcFile.close();

  return tracks_list; // 返回所有帧的轨迹数据
}

// 从文件中获取Lidar观测数据，世界坐标系下的数据，转换到自车传感器坐标系下
std::vector<LidarFrame> GetDetectionsFromFile(const std::string& scene_name) {
  // 构造Lidar观测文件的路径
  const std::string lidar_detection_filename =
      "../data/" + scene_name + "_lidar_detections.dat";
  std::ifstream srcFile(lidar_detection_filename, std::ios::in);
  if (!srcFile.is_open()) {
    std::cout << "Fail to open " << lidar_detection_filename << std::endl;
    return std::vector<LidarFrame>();
  }

  int frames_num;
  srcFile >> frames_num;// 从文件中读取帧数
  std::vector<LidarFrame> lidar_frames;// 存储Lidar帧数据
  lidar_frames.reserve(frames_num);// 预留空间，提高效率
  // 遍历每一帧的Lidar观测数据
  for (int i = 0; i < frames_num; ++i) {
    uint32_t frame_index = 0, detections_num = 0;
    double timestamp = 0;
    srcFile >> frame_index >> detections_num >> timestamp;// 帧索引和观测数

    double ego_x, ego_y, ego_yaw;// 目标车的位置信息
    srcFile >> ego_x >> ego_y >> ego_yaw;// 读取目标车的位置信息
    const Vec2d ego_position(ego_x, ego_y);// 目标车位置

    std::vector<LidarDetection> detections;
    detections.reserve(detections_num);
    // 遍历当前帧的所有观测数据
    for (int j = 0; j < detections_num; ++j) {
      uint32_t id;// 观测ID
      double x, y, z, l, w, h, yaw;// 观测的位置信息和尺寸信息
      srcFile >> id >> x >> y >> z >> l >> w >> h >> yaw;// 读取观测信息
      // 创建LidarDetection对象，并计算与自车的距离(世界坐标系下)
      detections.emplace_back(id, timestamp, Vec3d(x, y, z), Vec3d(l, w, h),
                              yaw);
      detections.back().GetDistanceToEgo(ego_position);
    }
    // 目标车位置从世界坐标系到传感器坐标系的变换，正常的旋转矩阵是传感器坐标系的点到世界坐标的，这里用其逆矩阵进行变换
    //这是一个变换类
    const Transformation2d world_to_vehicle(ego_position, ego_yaw);
     // 创建Lidar帧并添加到列表中
    lidar_frames.emplace_back(frame_index, timestamp, world_to_vehicle,
                              detections);
  }
  srcFile.close();

  return lidar_frames;
}

int main(int argc, char** argv) {
  std::cout << "Multiple Object Tracking Based on Lidar Detections.\n";
// 获取场景名，如果命令行参数有传递则使用，否则默认为"scene-0061"
  const std::string scene_name = argc > 1 ? argv[1] : "scene-0061";
  std::cout << "scene_name : " << scene_name << "\n";
// 获取Lidar帧数据
  const std::vector<LidarFrame> lidar_frames =
      GetDetectionsFromFile(scene_name);
// 初始化Tracker对象，并指定输出文件名
  Tracker tracker(GetOutputFilename(scene_name));
  std::vector<std::vector<Track>> published_tracks_list;// 用于存储每一帧的发布轨迹数据
// 对每一帧数据执行追踪
  for (const LidarFrame& frame : lidar_frames) {
    tracker.Run(frame);// 执行追踪
    published_tracks_list.push_back(tracker.PublishTracks(frame));// 获取并保存当前帧发布的轨迹
  }
// 获取地面真值轨迹数据
  std::vector<std::vector<Track>> gt_tracks_list = GetGroundTruthTracksFromFile(scene_name);
// 计算MOTA（Multiple Object Tracking Accuracy）
  const double mota = PerformanceEvaluation(published_tracks_list, gt_tracks_list);
  return 0;
}