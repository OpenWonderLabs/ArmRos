#pragma once

#include "woan_platform.h"      // ⚡ 包含平台抽象 + DLL export/import
#include "woan_api/woan_service.h"
#include "woan_api/woan_define.h"
#include "woan_api/woan_core.h"
#include "woan_define.h"

#include <vector>
#include <cstring>
#include <algorithm>
#include <new>

namespace woan_api {

/**
 * @class WoanAPI
 * @brief Main interface for WoanArm SO library
 *
 * Provides Move API (MoveJ, MoveL, MoveP) for robot control
 */
class WOAN_CPP_API WoanAPI {
public:
    static woan_robot_handle* create_robot(const woan_config_t& config);
    static void destroy_robot(woan_robot_handle* handle);

    static int enable_motors(woan_robot_handle* handle);
    static int disable_motors(woan_robot_handle* handle);

    static int movej(woan_robot_handle* handle,
                     const JointArray& target,
                     double speed_scale = 1.0,
                     uint8_t trajectory_connect = 0);

    static int movel(woan_robot_handle* handle,
                     const Pose& pose,
                     double speed_scale = 1.0,
                     uint8_t trajectory_connect = 0);


    static int movep(woan_robot_handle* handle,
                     const Pose& pose,
                     double speed_scale = 1.0,
                     uint8_t trajectory_connect = 0);

    static int execute_buffered_trajectory(woan_robot_handle* handle);
    static int clear_trajectory_buffer(woan_robot_handle* handle);

    static JointArray get_joint_positions(woan_robot_handle* handle);
    static JointArray get_joint_positions_from_motors(woan_robot_handle* handle);
    static JointArray get_joint_velocities(woan_robot_handle* handle);
    static ArmStateFromMotor get_arm_state_from_motor(woan_robot_handle* handle);
    static Pose get_end_effector_pose(woan_robot_handle* handle);

    static int send_trajectory_point(woan_robot_handle* handle,
                                     const JointArray& positions,
                                     const JointArray& velocities);

    // Overload that accepts accelerations for compatibility; accelerations are currently ignored.
    static int send_trajectory_point(woan_robot_handle* handle,
                                     const JointArray& positions,
                                     const JointArray& velocities,
                                     const JointArray& accelerations);

    static int ArmGravityCompensation(woan_robot_handle* handle);
    static void StopGravityCompensation(woan_robot_handle* handle);

    static int send_trajectory(woan_robot_handle* handle,
                               const std::vector<TrajectoryPoint>& trajectory);

    static int cancel_trajectory(woan_robot_handle* handle);
    static bool is_hardware_connected(woan_robot_handle* handle);
};

} // namespace woan_api
