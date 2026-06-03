#pragma once

#include <pinocchio/fwd.hpp>
#include "woan_api/encryption.h"
#include "woan_api/woan_platform.h"
#include "types.h"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/collision/collision.hpp>

#include <Eigen/Dense>
#include <string>
#include <memory>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <vector>
#include <cstdint>
#include <regex>

namespace woan {

class WOAN_CPP_API RobotModel {
public:
  RobotModel();
  ~RobotModel();

  /**
   * 加载模型文件，至少包含urdf文件和3D模型文件，可选包含srdf文件
   * model_path: 模型文件路径
   * model_name: urdf文件名，不带后缀
   * srdf_name: 可选，srdf文件名或绝对路径。如果为空，则默认在model_path下寻找model_name.srdf
   */
  void loadModel(const std::string &model_path, const std::string &model_name, const std::string &srdf_name = "");

  /**
   * 更新模型变换矩阵，q为关节角度
   */
  void updatePlacements(const Eigen::VectorXd &q);

  /**
   * 正运动学，更新位姿
   */
  void forwardKinematics(const Eigen::VectorXd &q);

  /**
   * 得到末端位姿，jingyi
   */
  Eigen::Matrix4d getEndEffectorPose(const Eigen::VectorXd &q) const;
  /**
   * 逆运动学，求解joints
   */
  bool inverseKinematics(const Transform &transform, Eigen::VectorXd &q_out,
                         int joint_id = -1);

  /**
   * 逆运动学，求解joints，输入一个当前关节位置作为起始点
   */
  bool inverseKinematics(const Transform &transform,
                         const Eigen::VectorXd &q_in, Eigen::VectorXd &q_out,
                         int joint_id = -1);

  /**
   * 计算指定关节的雅可比矩阵 (6xnv)
   */
  Eigen::MatrixXd getJacobian(const Eigen::VectorXd &q, int joint_id = -1);

  /**
   * 根据末端笛卡尔速度计算关节速度 (使用阻尼最小二乘伪逆)
   */
  Eigen::VectorXd computeJointVelocities(const Eigen::VectorXd &q,
                                         const Eigen::VectorXd &v_cartesian,
                                         int joint_id = -1);

  /**
   * 计算给定关节位置(q)、速度(dq)、加速度(ddq)下的关节力矩
   */
  Eigen::VectorXd computeTorques(const Eigen::VectorXd &q,
                                 const Eigen::VectorXd &dq,
                                 const Eigen::VectorXd &ddq);

  /**
   * 获取关节位姿
   */
  Transform getTransform(int joint);

  /**
   * 检测模型是否自碰撞，检测前记得先updatePlacements
   */
  bool isCollision(bool stopAtFirstCollision = true);

  /**
   * 检测模型是否自碰撞
   */
  bool isCollision(const Eigen::VectorXd &q, bool stopAtFirstCollision = true);

  /**
   * 返回关节转动范围下限，如果在URDF中定义了limit，这里可以读到
   */
  const Eigen::VectorXd &lowerJointLimits() const;

  /**
   * 返回关节转动范围上限，如果在URDF中定义了limit，这里可以读到
   */
  const Eigen::VectorXd &upperJointLimits() const;

  /**
   * 打印模型信息（关节名称和限位）
   */
  void printModelInfo();

  /**
   * 打印关节状态信息（规划位置、实际位置、规划速度、实际速度、规划力矩、实际力矩）
   * @param planned_position 规划位置
   * @param motor_position 实际位置（电机位置）
   * @param planned_velocity 规划速度
   * @param motor_velocity 实际速度（电机速度）
   * @param planned_torque 规划力矩
   * @param motor_torque 实际力矩（电机力矩）
   */
  void printJointState(const Eigen::VectorXd& planned_position,
                       const Eigen::VectorXd& motor_position,
                       const Eigen::VectorXd& planned_velocity,
                       const Eigen::VectorXd& motor_velocity,
                       const Eigen::VectorXd& planned_torque,
                       const Eigen::VectorXd& motor_torque);

  /**
   * 打印关节状态信息（仅位置和速度，不包含力矩）
   * @param planned_position 规划位置
   * @param motor_position 实际位置（电机位置）
   * @param planned_velocity 规划速度
   * @param motor_velocity 实际速度（电机速度）
   */
  void printJointState(const Eigen::VectorXd& planned_position,
                       const Eigen::VectorXd& motor_position,
                       const Eigen::VectorXd& planned_velocity,
                       const Eigen::VectorXd& motor_velocity);

  /**
   * 活动关节数量
   */
  int njoints;

  /**
   * 是否自碰撞
   */
  bool isSelfCollision(bool stopAtFirstCollision = true);

  /**
   * 四元数转换
   */
  Eigen::Quaterniond rpy2quat(double roll, double pitch, double yaw);

  /**
   * 获取零点偏差向量
   */
  const Eigen::VectorXd& getZeroBias() const { return zero_bias_; }

private:
  pinocchio::Model model_;
  pinocchio::GeometryModel geom_model_;
  std::unique_ptr<pinocchio::Data> data_ptr_;
  std::unique_ptr<pinocchio::GeometryData> geom_data_ptr_;
  std::string model_path_;
  std::string urdf_filename_;
  std::string srdf_filename_;
  
  Eigen::VectorXd zero_bias_;  // 零点偏差向量（用于角度转换）
};

} // namespace woan
