#pragma once

#include "woan_api/woan_platform.h"

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <algorithm>
#include <cstdlib>
#include <limits.h>

namespace woan_api {

/**
 * @brief 轨迹数据采集工具类
 * 专门用于采集规划后的 关节位置、速度、加速度、加加速度
 */
class TrajectoryLogger {
public:
    /**
     * @brief 保存规划后的轨迹数据到 CSV 文件
     * @param api_name 调用的 API 名称 (如 "movej", "movel", "movep")
     * @param positions 规划的关节位置序列
     * @param velocities 规划的关节速度序列
     * @param accelerations 规划的关节加速度序列
     * @param dt 控制周期 (s)
     */
    static void savePlannedData(const std::string& api_name,
                               const std::vector<Eigen::VectorXd>& positions,
                               const std::vector<Eigen::VectorXd>& velocities,
                               const std::vector<Eigen::VectorXd>& accelerations,
                               double dt,
                               const std::vector<Eigen::VectorXd>& jerks = {});
};

} // namespace woan_api