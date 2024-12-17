# Lidar-based-Multi-Target-Tracking
Implement multi-target tracking based on the LiDAR data from the nuScenes dataset. Use the Kalman Filter for state estimation and the Nearest Neighbor (NN) matching method for association. Evaluate the tracking performance using the MOTA metric and visualize the tracking results in the BEV (Bird's Eye View) perspective.

## 跟踪结果

![功能测试](https://github.com/allrivertosea/Lidar-based-Multi-Target-Tracking/blob/main/visualization/output_video.gif)

## 环境说明

- Eigen 3.4.0
- Opencv 4.5.5
- Cmake 3.16.3

## 使用说明

```
mkdir build
cd build
cmake ..
make -j8
./lidar_mot scene-xxxx
```
