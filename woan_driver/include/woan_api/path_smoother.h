#pragma once

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

// 五次多项式类
class QuinticPolynomial {
private:
  double a0, a1, a2, a3, a4, a5; // 多项式系数

public:
  // 构造函数：根据边界条件计算五次多项式系数
  QuinticPolynomial(double x0, double v0, double a0_acc, double x1, double v1,
                    double a1_acc, double T);

  // 计算位置
  double position(double t) const;

  // 计算速度
  double velocity(double t) const;

  // 计算加速度
  double acceleration(double t) const;
};

// 路径点结构
struct PathPoint {
  double position;
  double velocity;
  double acceleration;
  double time;

  PathPoint(double pos = 0, double vel = 0, double acc = 0, double t = 0)
      : position(pos), velocity(vel), acceleration(acc), time(t) {}
};

// 机械臂路径平滑器
class PathSmoother {
private:
  std::vector<PathPoint> waypoints;
  std::vector<QuinticPolynomial> polynomials;

public:
  // 添加路径点
  void addWaypoint(double position, double velocity = 0,
                   double acceleration = 0, double time = 0);

  // 生成平滑路径
  void generateSmoothPath();

  // 计算指定时间的位置、速度、加速度
  PathPoint interpolate(double t) const;

  double calculateSCurveTime(double distance, double max_vel, double max_acc, double max_j);

  // 清空路径点
  void clear();
};
