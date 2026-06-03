#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include <iostream>
#include <cstring>
#include <cstddef>

namespace woan_api {

/**
 * @brief 三次样条插值类
 * 用于将稀疏的via point平滑插值到更高频率的轨迹
 * 实现了基于三对角矩阵求解（追赶法/Thomas Algorithm）的三次样条插值
 */
class CubicSpline {
public:
    /**
     * @brief 边界条件类型
     */
    enum BoundType {
        BoundType_First_Derivative = 1,   // 一阶导数边界条件（指定首尾速度），也称Clamped边界
        BoundType_Second_Derivative = 2   // 二阶导数边界条件（指定首尾加速度），也称Natural边界(若为0)
    };

    CubicSpline() = default;
    ~CubicSpline() = default;

    /**
     * @brief 加载via point数据并计算样条参数 (兼容旧接口)
     * @param x_data 时间数组（自变量，单调递增）
     * @param y_data 位置数组（因变量）
     * @param count 数据点数量
     * @param bound1 左边界条件值（起始速度或加速度）
     * @param bound2 右边界条件值（结束速度或加速度）
     * @param type 边界条件类型
     * @return 成功返回true，失败返回false
     */
    bool loadData(const double *x_data, const double *y_data, int count, 
                  double bound1, double bound2, BoundType type);

    /**
     * @brief 加载via point数据 (vector接口)
     */
    bool loadData(const std::vector<double>& x_data, const std::vector<double>& y_data,
                  double bound1, double bound2, BoundType type);

    /**
     * @brief 根据时间插值得到位置、速度、加速度
     * @param x_in 输入时间
     * @param y_out 输出位置
     * @param vel_out 输出速度 (一阶导)
     * @param acc_out 输出加速度 (二阶导)
     * @return 成功返回true
     */
    bool getYbyX(double x_in, double &y_out, double &vel_out, double &acc_out) const;

private:
    /**
     * @brief 求解三对角矩阵构建样条 (Thomas Algorithm)
     * 核心算法：求解 M (二阶导数) 向量
     */
    bool computeSpline(BoundType type, double bound1, double bound2);

private:
    std::vector<double> x_sample_; // 采样点 x (时间)
    std::vector<double> y_sample_; // 采样点 y (位置)
    std::vector<double> M_;        // 求解出的二阶导数值 (弯矩)
};

} // namespace woan_api
