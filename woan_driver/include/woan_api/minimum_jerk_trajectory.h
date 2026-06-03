#pragma once

#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

/**
 * @brief 最小加加速度轨迹生成器
 * 
 * 基于五次多项式实现最小加加速度轨迹规划，确保位置、速度、
 * 加速度和加加速度的连续性。
 */
class MinimumJerkTrajectory {
public:
  struct AxisWaypoint {
    double start_position{0.0};
    double end_position{0.0};
    double start_velocity{0.0};
    double end_velocity{0.0};
    double start_acceleration{0.0};
    double end_acceleration{0.0};
  };

  struct AxisSample {
    double position{0.0};
    double velocity{0.0};
    double acceleration{0.0};
    double jerk{0.0};
  };

  using WaypointSequence = std::vector<AxisWaypoint>;
  using SampleSequence = std::vector<AxisSample>;

  MinimumJerkTrajectory() = default;
  MinimumJerkTrajectory(WaypointSequence waypoints, double duration);

  SampleSequence sample(double t) const;
  double duration() const { return duration_; }
  std::size_t dof() const { return coefficients_.size(); }

private:
  struct AxisCoefficients {
    double a0{0.0};
    double a1{0.0};
    double a2{0.0};
    double a3{0.0};
    double a4{0.0};
    double a5{0.0};
  };

  void buildCoefficients();
  AxisCoefficients calculateCoefficients(const AxisWaypoint &wp) const;
  AxisSample evaluate(const AxisCoefficients &coeff, double t) const;

  WaypointSequence waypoints_;
  std::vector<AxisCoefficients> coefficients_;
  double duration_{1.0};
};

