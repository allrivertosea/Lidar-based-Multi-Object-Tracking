#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>

// 2D 向量类
class Vec2d {
 public:
  Vec2d() : x_(0.0), y_(0.0) {}// 默认构造函数，初始化为 (0.0, 0.0)
  Vec2d(double x, double y) : x_(x), y_(y) {}// 构造函数，使用指定的 x 和 y 值初始化

  double x() const { return x_; }// 获取 x 分量
  double y() const { return y_; }// 获取 y 分量

  double Length() const { return std::sqrt(x_ * x_ + y_ * y_); }// 计算向量的长度（模）
  // 重载加法运算符，实现向量相加、相减和与标量相乘
  Vec2d operator+(const Vec2d& b) const {
    return Vec2d(x_ + b.x(), y_ + b.y());
  }
  Vec2d operator-(const Vec2d& b) const {
    return Vec2d(x_ - b.x(), y_ - b.y());
  }
  Vec2d operator*(const double k) const { return Vec2d(k * x_, k * y_); }

 private:
  double x_;// x，y 分量
  double y_;
};

// 3D 向量类
class Vec3d {
 public:
  Vec3d() : x_(0.0), y_(0.0), z_(0.0) {}// 默认构造函数，初始化为 (0.0, 0.0, 0.0)
  Vec3d(double x, double y, double z) : x_(x), y_(y), z_(z) {}// 构造函数，使用指定的 x, y, z 值初始化
  Vec3d(const Eigen::Vector3d& p) : x_(p[0]), y_(p[1]), z_(p[2]) {}// 使用 Eigen::Vector3d 构造函数初始化

  double x() const { return x_; }// 获取三个分量
  double y() const { return y_; }
  double z() const { return z_; }

  Vec2d xy() const { return Vec2d(x_, y_); }// 提取 2D xy 分量

  double Length() const { return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_); }// 计算向量的长度（模）
  // 重载运算符，实现向量相加，相减和标量相乘
  Vec3d operator+(const Vec3d& b) const {
    return Vec3d(x_ + b.x(), y_ + b.y(), z_ + b.z());
  }
  Vec3d operator-(const Vec3d& b) const {
    return Vec3d(x_ - b.x(), y_ - b.y(), z_ - b.z());
  }
  Vec3d operator*(const double k) const {
    return Vec3d(k * x_, k * y_, k * z_);
  }

 private:
  double x_;// x,y,z 分量
  double y_;
  double z_;
};

// 2D 变换类，世界坐标系到自车传感器坐标系的变换
class Transformation2d {
 public:
  Transformation2d() = default;// 默认构造函数
  ~Transformation2d() = default;// 析构函数
  // 构造函数，使用平移向量和旋转角度初始化变换
  Transformation2d(const Vec2d translation, const double theta)
      : translation_(translation), theta_(theta) {}

  Vec2d translation() const { return translation_; }// 获取平移向量
  double theta() const { return theta_; }// 获取旋转角度

  // 变换函数，将一个 2D 点应用此变换，先进行平移，再进行旋转(逆变换，使用旋转矩阵的逆矩阵)
  Vec2d Transform(const Vec2d point) const {
    const Vec2d p = point - translation_;
    return Vec2d(p.x() * std::cos(theta_) + p.y() * std::sin(theta_),
                 p.y() * std::cos(theta_) - p.x() * std::sin(theta_));
  }

 private:
  Vec2d translation_;// 平移向量
  double theta_;     // 旋转角度（弧度）
};
