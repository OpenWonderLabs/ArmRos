#pragma once

#include "woan_interface.h"
#include "woan_define.h"
#include "woan_platform.h"
#include <memory>

namespace woan_api {

// Forward declaration - implementation is in woan_interface.cpp
class WoanCore;

/**
 * @class WoanService
 * @brief Service wrapper that bridges WoanAPI interface to WoanCore implementation
 * 
 * This class wraps the core control logic and provides access to the robot handle.
 */
class WOAN_CPP_API WoanService {
public:
    WoanService();
    ~WoanService();

    // Initialize with configuration
    bool initialize(const woan_config_t& config);

    // Cleanup
    void shutdown();

    // Motor control
    bool enable_motors();
    bool disable_motors();

    // Motion primitives
    int execute_movej(const JointArray& target,
                     double speed_scale,
                     uint8_t trajectory_connect);

    int execute_movel(const Pose& pose,
                     double speed_scale,
                     uint8_t trajectory_connect);

    int execute_movep(const Pose& pose,
                     double speed_scale,
                     uint8_t trajectory_connect);

    // Trajectory buffering
    int flush_buffered_trajectories();
    int clear_buffers();

    // State queries
    JointArray query_joint_positions();
    JointArray query_joint_positions_from_motors();  // 强制从电机读取
    JointArray query_joint_velocities();
    Pose query_end_effector_pose();

    // MoveIt integration
    int send_traj_point(const JointArray& positions, const JointArray& velocities);

    // Leader arm gravity compensation
    int ArmGravityCompensation();
    void StopGravityCompensation();

    // Hardware connection check
    bool is_hardware_connected() const;

    // Get arm state from motor
    ArmStateFromMotor query_arm_state_from_motor() const;

    // Send complete trajectory
    int send_trajectory(const std::vector<TrajectoryPoint>& trajectory);

    // Cancel ongoing trajectory
    int cancel_trajectory();

    // Get internal handle (for C API bridge)
    woan_robot_handle* get_handle() const { return handle_; }

private:
    woan_robot_handle* handle_;
    std::shared_ptr<WoanCore> core_;
    bool initialized_;
};

}  // namespace woan_api