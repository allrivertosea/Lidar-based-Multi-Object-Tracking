#include "visualizer.h"

#include <opencv4/opencv2/imgproc/imgproc_c.h>

namespace {

// 定义一些常用的颜色
const cv::Scalar kYellowColor = cv::Scalar(0, 255, 255); // 黄色
const cv::Scalar kGreenColor = cv::Scalar(0, 255, 0);   // 绿色
const cv::Scalar kRedColor = cv::Scalar(0, 0, 255);     // 红色
const cv::Scalar kBlueColor = cv::Scalar(255, 0, 0);    // 蓝色
const cv::Scalar kBlackColor = cv::Scalar(0, 0, 0);     // 黑色
const cv::Scalar kWhileColor = cv::Scalar(255, 255, 255); // 白色
const cv::Scalar kGrayColor = cv::Scalar(80, 80, 80);   // 灰色

}  // namespace

// 绘制跟踪结果的函数
void Visualizer::DrawTracks(const std::vector<Track>& tracks) {
  for (const Track& track : tracks) {
    // 如果目标尺寸过小，则跳过该目标
    if (track.size().x() < 0.7 && track.size().y() < 0.7) continue;
    // 如果目标未被确认，跳过
    if (!track.IsConfirmed()) continue;
    
    const std::vector<Vec2d> corners = track.GetCorners(); // 获取目标的角点坐标
    std::vector<cv::Point> corners_image; // 存储转换后的图像坐标
    cv::Point id_pos; // 用于标记ID的位置

    for (int i = 0; i < corners.size(); ++i) {
      // 将角点坐标从世界坐标系转换到车辆坐标系
      const Vec2d corner_vehicle = world_to_vehicle_.Transform(corners[i]);
      // 再将坐标从车辆坐标系转换到图像坐标系
      const cv::Point p = FromVehicleToImage(corner_vehicle);
      // 如果点超出了图像范围，则中断循环
      if (IsPointBeyondGridImage(p)) break;
      
      corners_image.push_back(p);
      // 记录第4个角点的位置用于放置ID
      if (i == 3) id_pos = p;
    }
    if (corners_image.size() < 4) continue; // 如果角点不足4个，跳过

    // 在图像上绘制目标边界框（多边形）
    cv::polylines(image_, corners_image, true, kRedColor, 1, 8, 0);
    // 在目标上绘制ID文本
    cv::putText(image_, std::to_string(track.id()),
                cv::Point(id_pos.x + 2, id_pos.y - 2), cv::FONT_HERSHEY_PLAIN,
                1, kRedColor, 1, CV_AA);
  }
}

// 绘制Ground Truth跟踪目标的函数
void Visualizer::DrawGTTracks(const std::vector<Track>& tracks) {
  for (const Track& track : tracks) {
    const std::vector<Vec2d> corners = track.GetCorners(); // 获取角点坐标
    std::vector<cv::Point> corners_image; // 存储转换后的图像坐标
    cv::Point id_pos; // 用于标记ID的位置

    for (int i = 0; i < corners.size(); ++i) {
      const Vec2d corner_vehicle = world_to_vehicle_.Transform(corners[i]);
      const cv::Point p = FromVehicleToImage(corner_vehicle);
      if (IsPointBeyondGridImage(p)) break;
      corners_image.push_back(p);
      if (i == 1) id_pos = p; // 记录第2个角点的位置用于放置ID
    }
    if (corners_image.size() < 4) continue;

    // 在图像上绘制GT边界框
    cv::polylines(image_, corners_image, true, kGreenColor, 1, 8, 0);
    // 在GT目标上绘制ID文本
    cv::putText(image_, std::to_string(track.id()),
                cv::Point(id_pos.x + 2, id_pos.y - 2), cv::FONT_HERSHEY_PLAIN,
                1, kGreenColor, 1, CV_AA);
  }
}

// 构造函数：初始化图像并调用绘制函数
Visualizer::Visualizer(const LidarFrame& frame,
                       const std::vector<Track>& tracks)
    : timestamp_(frame.timestamp()),
      world_to_vehicle_(frame.world_to_vehicle()),
      image_(cv::Mat(kImageHeight, kImageWidth, CV_8UC3, kBlackColor)) {
  DrawGrids();        // 绘制网格
  DrawCoordinates();  // 绘制坐标轴
  DrawTracks(tracks); // 绘制跟踪结果
  
  SaveImages(frame.index(), tracks.size()); // 保存结果图像
}

// 绘制坐标轴的函数
void Visualizer::DrawCoordinates() {
  // 显示时间戳
  cv::putText(image_, "T: " + std::to_string(timestamp_), cv::Point(20, 50),
              cv::FONT_HERSHEY_PLAIN, 2, kRedColor, 1, CV_AA);

  // 定义坐标轴的起点和终点
  const cv::Point frame_x_axis_min =
      FromVehicleToImage(Vec2d(kMaxBackRange, 0));
  const cv::Point frame_x_axis_max =
      FromVehicleToImage(Vec2d(kMaxFrontRange, 0));
  const cv::Point frame_y_axis_min =
      FromVehicleToImage(Vec2d(0, kMaxRightRange));
  const cv::Point frame_y_axis_max =
      FromVehicleToImage(Vec2d(0, kMaxLeftRange));

  // 绘制坐标轴
  cv::arrowedLine(image_, frame_x_axis_min, frame_x_axis_max, kRedColor, 2, 8,
                  0, 0.05);
  cv::arrowedLine(image_, frame_y_axis_min, frame_y_axis_max, kBlueColor, 2, 8,
                  0, 0.05);
  // 标注坐标轴标签
  cv::putText(image_, "x",
              cv::Point(frame_x_axis_max.x + 25, frame_x_axis_max.y + 25),
              cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, kRedColor, 1, CV_AA);
  cv::putText(image_, "y",
              cv::Point(frame_y_axis_max.x, frame_y_axis_max.y - 15),
              cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, kBlueColor, 1, CV_AA);

  // 标记30m和60m距离
  const cv::Point label_30m = FromVehicleToImage(Vec2d(30, 0));
  const cv::Point label_60m = FromVehicleToImage(Vec2d(60, 0));

  cv::putText(image_, "30m", cv::Point(label_30m.x + 5, label_30m.y),
              cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, kRedColor, 1, CV_AA);
  cv::putText(image_, "60m", cv::Point(label_60m.x + 5, label_60m.y),
              cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, kRedColor, 1, CV_AA);
}

// 绘制网格的函数
void Visualizer::DrawGrids() {
  const double base_size = 5.0;  // 网格基本单位（米）
  for (double row = kMaxBackRange; row <= kMaxFrontRange; row += base_size) {
    const int row_p = static_cast<int>((row - kMaxBackRange) / kGridSize);
    const cv::Point p1 = cv::Point(0, row_p);
    const cv::Point p2 = cv::Point(kImageWidth, row_p);
    cv::line(image_, p1, p2, kGrayColor, 1); // 绘制水平方向的网格线
  }
  for (double col = kMaxRightRange; col <= kMaxLeftRange; col += base_size) {
    const int col_p = static_cast<int>((col - kMaxRightRange) / kGridSize);
    const cv::Point p1 = cv::Point(col_p, 0);
    const cv::Point p2 = cv::Point(col_p, kImageHeight);
    cv::line(image_, p1, p2, kGrayColor, 1); // 绘制垂直方向的网格线
  }
}

// 保存图像为视频的函数
void Visualizer::SaveImages(const int frame_index, const int tracks_num) {
  static cv::VideoWriter video_writer;
  static bool is_initialized = false;
  const std::string video_filename = "../visualization/output_video.mp4";

  // 初始化视频写入器
  if (!is_initialized) {
    video_writer.open(video_filename, cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 10,
                      cv::Size(kImageWidth, kImageHeight));
    if (!video_writer.isOpened()) {
      std::cerr << "Error: Cannot open video file for writing." << std::endl;
      return;
    }
    is_initialized = true;
  }

  // 写入当前帧到视频文件
  video_writer.write(image_);
  std::cout << "Frame " << frame_index << " written to video." << std::endl;
}
