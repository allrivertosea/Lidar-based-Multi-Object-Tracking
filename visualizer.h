#pragma once

#include <opencv4/opencv2/opencv.hpp>
// If some complier errors occur on the above line, please try to change it to #include <opencv2/opencv.hpp>

#include "lidar_detection.h"
#include "math.h"
#include "track.h"

class Visualizer {
 public:
  Visualizer() = default;
  ~Visualizer() = default;

  Visualizer(const LidarFrame& frame, const std::vector<Track>& tracks);

  void DrawGrids();
  void DrawCoordinates();
  void DrawTracks(const std::vector<Track>& tracks);
  void DrawGTTracks(const std::vector<Track>& tracks);
  void SaveImages(const int frame_index, const int tracks_num);

 private:
  cv::Mat image_;
  double timestamp_;
  Transformation2d world_to_vehicle_;

  static constexpr double kMaxFrontRange = 70.0;
  static constexpr double kMaxBackRange = -40.0;
  static constexpr double kMaxLeftRange = 40.0;
  static constexpr double kMaxRightRange = -40.0;
  static constexpr double kGridSize = 0.1;

  static const int kSizeMargin = 0;
  static const int kImageHeight =
      (kMaxFrontRange - kMaxBackRange) / kGridSize + kSizeMargin;
  static const int kImageWidth =
      (kMaxLeftRange - kMaxRightRange) / kGridSize + kSizeMargin;

  static const int kXAxisOriginRow =
      kMaxFrontRange / kGridSize + kSizeMargin / 2;
  static const int kYAxisOrigincol = kImageWidth / 2;

  cv::Point FromVehicleToImage(const Vec2d& point) {
    const int x_row_offset =
        kXAxisOriginRow - static_cast<int>(point.x() / kGridSize);
    const int y_col_offset =
        kYAxisOrigincol - static_cast<int>(point.y() / kGridSize);
    return cv::Point(y_col_offset, x_row_offset);
  }

  bool IsPointBeyondGridImage(const cv::Point& p) const {
    return p.x < 0 || p.x >= kImageWidth || p.y < 0 || p.y >= kImageHeight;
  }
};