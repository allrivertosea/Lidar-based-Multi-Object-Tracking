
#include "track.h"

// 设置一些常量，用于初始化系统过程噪声协方差矩阵，后验估计协方差矩阵，观测噪声协方差矩阵
//包括：系统方差（噪声），预测方差(预测噪声)，观测方差(观测噪声）
namespace {
static constexpr double kInitPositionVariance = 1.0;
static constexpr double kInitVelocityVariance = 4.0;
static constexpr double kInitAccelerationVariance = 1.0;
static constexpr double kInitYawVariance = 0.01;
static constexpr double kInitYawRateVariance = 0.01;

static constexpr double kPredictPositionVariance = 0.25;
static constexpr double kPredictVelocityVariance = 1.0;
static constexpr double kPredictAccelerationVariance = 0.25;
static constexpr double kPredictYawVariance = 0.01;
static constexpr double kPredictYawRateVariance = 0.01;

static constexpr double kLidarPositionNoiseVariance = 0.25;// 激光雷达位置噪声方差
static constexpr double kLidarYawNoiseVariance = 0.01;// 激光雷达偏航角噪声方差

static constexpr double kSmoothFilterCoefficient = 0.7;// 平滑滤波系数

static constexpr double kReliableDetectionRange = 50.0;  // 激光雷达可靠检测范围 (米)

Vec2d RotateByTheta(const Vec2d p, const double theta) {
  // 旋转坐标点 p，旋转角度为 theta，通过theta实现2D点p从世界坐标到自车传感器坐标系的转换
  return Vec2d(p.x() * std::cos(theta) + p.y() * std::sin(theta),
               p.y() * std::cos(theta) - p.x() * std::sin(theta));
}

}  // namespace

// 初始化下一个跟踪目标 ID
uint32_t Track::next_track_id_ = 1;

// 构造函数，初始化跟踪目标
Track::Track(const LidarDetection& detection)
    : id_(next_track_id_++),// 每产生一个新的跟踪目标分配一个新的ID
      created_timestamp_(detection.timestamp()),// 记录创建时间戳
      timestamp_(detection.timestamp()),// 当前时间戳
      last_update_timestamp_(detection.timestamp()),// 上次更新时间戳
      position_(detection.position()),// 初始化位置
      size_(detection.size()),// 初始化目标尺寸
      yaw_(detection.yaw()),// 初始化偏航角
      associated_lidar_detections_cnt_(1) // 激光雷达检测关联计数
    {
        velocity_ = Vec3d();// 初始化速度
        acceleration_ = Vec3d(); // 初始化加速度
        // 初始状态向量，包括位置、速度、加速度，所有速度和加速度初始化为0
        const Eigen::Matrix<double, 9, 1>& state{
            position_.x(), position_.y(), position_.z(), 0.0, 0.0,
            0.0,           0.0,           0.0,           0.0};
        // 初始系统过程噪声协方差矩阵，指定位置、速度、加速度的不确定性分别为1.0，4.0，1.0
        Eigen::Matrix<double, 9, 9> covariance = Eigen::Matrix<double, 9, 9>::Zero();
        covariance(0, 0) = covariance(1, 1) = covariance(2, 2) =
            kInitPositionVariance;
        covariance(3, 3) = covariance(4, 4) = covariance(5, 5) =
            kInitVelocityVariance;
        covariance(6, 6) = covariance(7, 7) = covariance(8, 8) =
            kInitAccelerationVariance;
        //使用状态向量state和系统过程噪声协方差矩阵初始化卡尔曼滤波器(基于运动模型)
        motion_kf_.set_state(state);
        motion_kf_.set_covariance(covariance);
        //航向角的跟踪和上面的分开，单独有一个卡尔曼滤波器
        yaw_rate_ = 0.0;
        const Eigen::Matrix<double, 2, 1>& yaw_state{yaw_, yaw_rate_};
        Eigen::Matrix<double, 2, 2> yaw_covariance =
            Eigen::Matrix<double, 2, 2>::Zero();
        yaw_covariance(0, 0) = kInitYawVariance;
        yaw_covariance(1, 1) = kInitYawRateVariance;
        // 初始化偏航卡尔曼滤波器
        yaw_kf_.set_state(yaw_state);
        yaw_kf_.set_covariance(yaw_covariance);
}

// 预测下一状态（位置、速度、加速度、偏航角等）
void Track::Predict(const double timestamp) {
  if (timestamp <= timestamp_) return;// 如果时间戳不合法，则不做预测
  const double dt = timestamp - timestamp_;// 计算时间间隔
  const double dt2 = dt * dt;

  timestamp_ = timestamp;// 更新当前时间戳
  // 9x9状态转移矩阵，表示状态如何从上一个时刻转移到当前时刻，由运动方程得到
  Eigen::Matrix<double, 9, 9> transition_matrix;
  transition_matrix << 1, 0, 0, dt, 0, 0, dt2, 0, 0, 0, 1, 0, 0, dt, 0, 0, dt2,
      0, 0, 0, 1, 0, 0, dt, 0, 0, dt2, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0,
      1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
  // 9x9预测噪声协方差矩阵（后验估计噪声协方差矩阵）,位置、速度、加速度分别为0.25，1.0，0.25
  Eigen::Matrix<double, 9, 9> predict_noise_cov = Eigen::Matrix<double, 9, 9>::Zero();
  predict_noise_cov(0, 0) = predict_noise_cov(1, 1) = predict_noise_cov(2, 2) = kPredictPositionVariance;
  predict_noise_cov(3, 3) = predict_noise_cov(4, 4) = predict_noise_cov(5, 5) = kPredictVelocityVariance;
  predict_noise_cov(6, 6) = predict_noise_cov(7, 7) = predict_noise_cov(8, 8) = kPredictAccelerationVariance;
  // 使用卡尔曼滤波器进行预测，基于状态转移矩阵和预测噪声协方差矩阵
  motion_kf_.Predict(transition_matrix, predict_noise_cov);
  // 得到跟踪目标的预测状态（位置、速度、加速度）
  position_ = Vec3d(motion_kf_.state()[0], motion_kf_.state()[1],
                    motion_kf_.state()[2]);
  velocity_ = Vec3d(motion_kf_.state()[3], motion_kf_.state()[4],
                    motion_kf_.state()[5]);
  acceleration_ = Vec3d(motion_kf_.state()[6], motion_kf_.state()[7],
                        motion_kf_.state()[8]);
  // 偏航角状态转移矩阵
  const Eigen::Matrix<double, 2, 2> yaw_transition_matrix{{1, dt}, {0, 1}};
  // 偏航预测噪声协方差矩阵
  Eigen::Matrix<double, 2, 2> yaw_predict_noise_cov{
      {kPredictYawVariance, 0}, {0, kPredictYawRateVariance}};
  // 使用偏航卡尔曼滤波器进行预测，得到预测状态
  yaw_kf_.Predict(yaw_transition_matrix, yaw_predict_noise_cov);
  yaw_ = yaw_kf_.state()[0];
  yaw_rate_ = yaw_kf_.state()[1];
}

// 更新跟踪目标的状态，基于新的激光雷达检测结果
void Track::Update(const LidarDetection& detection) {
  //观测向量3X1
  const Eigen::Matrix<double, 3, 1> measurement{detection.position().x(),
                                                detection.position().y(),
                                                detection.position().z()};
  // 如果距离较远，增加噪声
  const double noise_scale = detection.distance_to_ego() < kReliableDetectionRange ? 1.0 : 2.0;
  // 位置观测噪声协方差矩阵 3x3
  const Eigen::Matrix<double, 3, 3> measure_noise_cov = noise_scale * kLidarPositionNoiseVariance * Eigen::Matrix<double, 3, 3>::Identity();
  // 3x9位置观测矩阵,用于将状态映射到观测
  Eigen::Matrix<double, 3, 9> measure_matrix = Eigen::Matrix<double, 3, 9>::Zero();
  measure_matrix.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  // 使用运动卡尔曼滤波器进行状态更新，基于观测向量，观测噪声和观测矩阵
  motion_kf_.Update<3>(measurement, measure_noise_cov, measure_matrix);
  // 更新跟踪目标的状态
  position_ = Vec3d(motion_kf_.state()[0], motion_kf_.state()[1],
                    motion_kf_.state()[2]);
  velocity_ = Vec3d(motion_kf_.state()[3], motion_kf_.state()[4],
                    motion_kf_.state()[5]);
  acceleration_ = Vec3d(motion_kf_.state()[6], motion_kf_.state()[7],
                        motion_kf_.state()[8]);
  // 更新偏航角状态
  const Eigen::Matrix<double, 1, 1> yaw_measurement{detection.yaw()};
  const Eigen::Matrix<double, 1, 1> yaw_measure_noise_cov{
      noise_scale * kLidarYawNoiseVariance};
  Eigen::Matrix<double, 1, 2> yaw_measure_matrix{1, 0};
  yaw_kf_.Update<1>(yaw_measurement, yaw_measure_noise_cov, yaw_measure_matrix);
  yaw_ = yaw_kf_.state()[0];
  yaw_rate_ = yaw_kf_.state()[1];
  // 对目标尺寸进行平滑滤波，尺寸一般不会变化
  size_ = size_ * kSmoothFilterCoefficient + detection.size() * (1 - kSmoothFilterCoefficient);

  last_update_timestamp_ = detection.timestamp();// 更新最后一次更新时间戳
  ++associated_lidar_detections_cnt_;// 增加该跟踪目标关联到检测次数
}

//丢失逻辑判断：关联次数超过2次阈值，那么为1.5，否则为0.1，计算当前时间和上次更新时间的差值，也就是多久没更新，如果时间大于丢失时间门限值，则该跟踪目标已丢失
bool Track::IsLost() const {
  const double kTrackLostTimeGating = IsConfirmed() ? 1.5 : 0.1;  // second
  return timestamp_ - last_update_timestamp_ > kTrackLostTimeGating;
}
// 判断目标是否已确认，关联次数大于2次认为目标已确认
bool Track::IsConfirmed() const { return associated_lidar_detections_cnt_ > 2; }

// 获取目标的四个角点，用于后续可视化或其它用途
std::vector<Vec2d> Track::GetCorners() const {
  const double theta = -yaw_;// 偏航角取负值
  const Vec2d center(position_.x(), position_.y());// 目标中心点
  const Vec2d tl_shift(0.5 * size_.y(), 0.5 * size_.x());//长方体的角点可以通过其中心点加上角点的偏移向量得到，左上、右上、左下、右下
  const Vec2d tr_shift(0.5 * size_.y(), -0.5 * size_.x());
  const Vec2d bl_shift(-0.5 * size_.y(), 0.5 * size_.x());
  const Vec2d br_shift(-0.5 * size_.y(), -0.5 * size_.x());
  const Vec2d top_left = center + RotateByTheta(tl_shift, theta);// 旋转每个角点
  const Vec2d top_right = center + RotateByTheta(tr_shift, theta);
  const Vec2d bottom_left = center + RotateByTheta(bl_shift, theta);
  const Vec2d bottom_right = center + RotateByTheta(br_shift, theta);
  return std::vector<Vec2d>{top_left, bottom_left, bottom_right, top_right};// 返回BEV视图下的四个角点
}