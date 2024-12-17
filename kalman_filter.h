#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

// KalmanFilter 模板类，XN 表示状态向量的维度
template <uint32_t XN>
class KalmanFilter {
 public:
  KalmanFilter() = default;// 默认构造函数
  // 带初始状态和协方差的构造函数
  KalmanFilter(const Eigen::Matrix<double, XN, 1>& state,
               const Eigen::Matrix<double, XN, XN>& covariance)
      : state_(state), covariance_(covariance) {}
  // 设置状态向量
  void set_state(const Eigen::Matrix<double, XN, 1>& state) { state_ = state; }
  // 设置协方差矩阵
  void set_covariance(const Eigen::Matrix<double, XN, XN>& covariance) {
    covariance_ = covariance;
  }
  // 获取当前状态向量
  const Eigen::Matrix<double, XN, 1>& state() const { return state_; }
  // 获取当前协方差矩阵
  const Eigen::Matrix<double, XN, XN>& covariance() const {
    return covariance_;
  }
  // 预测函数， 输入：- transition_matrix: 状态转移矩阵，- prediction_cov: 预测噪声协方差矩阵，用于描述状态如何从 t 到 t+1 演变
  void Predict(const Eigen::Matrix<double, XN, XN>& transition_matrix,
               const Eigen::Matrix<double, XN, XN>& prediction_cov) {
    // 预测状态： x' = F * x，使用状态转移矩阵和上一时刻最优状态估计值
    state_ = transition_matrix * state_;
    // 预测协方差： P' = F * P * F^T + Q，使用状态转移矩阵和上一时刻最优状态估计值和预测噪声协方差矩阵（后验估计协方差矩阵）
    covariance_ = transition_matrix * covariance_ * transition_matrix.transpose() + prediction_cov;
  }
  // 更新函数（模板支持可变观测维度 ZN）
  // 输入：
  // - measurement: 当前的观测值向量
  // - measure_noise_cov: 测量噪声协方差矩阵
  // - measure_matrix: 测量矩阵，将状态向量映射到观测空间
  template <uint32_t ZN>
  void Update(const Eigen::Matrix<double, ZN, 1>& measurement,
              const Eigen::Matrix<double, ZN, ZN>& measure_noise_cov,
              const Eigen::Matrix<double, ZN, XN>& measure_matrix) {
    // 计算中间误差： y = z - H * x',观测值和观测矩阵和预测状态。该结果后续用于更新状态。
    const Eigen::Matrix<double, ZN, 1> innovation = measurement - measure_matrix * state_;
    // 计算中间协方差矩阵： S = H * P' * H^T + R，使用观测矩阵，预测协方差矩阵和观测噪声。该结果的逆后续用于计算卡尔曼增益
    const Eigen::Matrix<double, ZN, ZN> innovation_covariance =
        measure_matrix * covariance_ * measure_matrix.transpose() +
        measure_noise_cov;
    // 计算中间协方差的逆： S^-1，为了保证数值稳定，加上一个很小的对角项（1e-6）
    const Eigen::Matrix<double, ZN, ZN> innovation_covariance_inv =
        (innovation_covariance +
         Eigen::Matrix<double, ZN, ZN>::Identity() * (1e-6)).inverse();
    // 计算卡尔曼增益： K = P' * H^T * S^-1，预测协方差矩阵，观测矩阵的转置，中间协方差矩阵的逆
    const Eigen::Matrix<double, XN, ZN> kalman_gain =
        covariance_ * measure_matrix.transpose() * innovation_covariance_inv;
    // 更新状态： x = x' + K * y。等于预测状态+卡尔曼增益*预测和观测的误差（中间误差)，其实就是一种修正
    state_ += kalman_gain * innovation;
    // 更新协方差： P = P' - K * H * P'，等于预测协方差矩阵 - （卡尔曼增益*观测矩阵*预测协方差矩阵），其实就是一种修正
    covariance_ -= kalman_gain * measure_matrix * covariance_;
  }

 private:
  Eigen::Matrix<double, XN, 1> state_;// 状态向量
  Eigen::Matrix<double, XN, XN> covariance_;// 预测协方差矩阵，描述状态的估计误差
};
