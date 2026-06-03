#pragma once
#include <iostream>
#include <fstream>
#include "woan_api/BiQuadFilter.h"



#define DTOF 7

#define left_master_arm_ID      1
#define right_master_arm_ID     2
#define left_slave_arm_ID       3
#define right_slave_arm_ID      4

class ArmDynamics
{



public:
    ArmDynamics( int arm_type,
                int n_joints = DTOF, 
                double control_freq = 100,
                double pos_cutoff = 10.0,
                double vel_cutoff = 10.0,
                double acc_cutoff = 10.0 );
    //int CalTotalTorque(Eigen::VectorXd jpos, double link3_lenght,double link6_lenght,Eigen::VectorXd&  TotalTorque_total);
    int CalTotalTorque(Eigen::VectorXd jpos,double link3_lenght,double link6_lenght, Eigen::VectorXd& TotalTorque_total); 



private:

    // void CalBaseRecursor(const Eigen::VectorXd &jpos, const Eigen::VectorXd &jvel,const Eigen::VectorXd &jacc,
    //                    Eigen::Matrix<double, 6 * DTOF, 12 * DTOF >&_Wmat, const bool grav, const bool friction);


    struct DynamicsParams
    {
        int params_num;
        Eigen::VectorXi  params_idx;
        Eigen::VectorXd  params_set;
        //Vector3x1 _gravity;
    };


    Eigen::Matrix3d skew(const Eigen::Vector3d& v);
    Eigen::MatrixXd Vec2Icross(const Eigen::Vector3d &vec);
    Eigen::Matrix<double, 6, 6> SerialMat(std::vector<Eigen::MatrixXd>& T, int midx, int nidx);
    double VelocitySign(double x, int joint_type);
    Eigen::VectorXd CalTorque(const Eigen::VectorXd& jpos,  const Eigen::VectorXd& jvel,const Eigen::VectorXd& jacc,
                              const bool grav, const bool friction);
    void CalBaseRecursor(const Eigen::VectorXd &jpos, const Eigen::VectorXd &jvel,const Eigen::VectorXd &jacc,
                       Eigen::Matrix<double, 6 * DTOF, 12 * DTOF >&_Wmat, const bool grav, const bool friction);
    Eigen::VectorXd CalTorqueGravity(Eigen::VectorXd& jpos);
    Eigen::VectorXd CalTorqueFriction(Eigen::VectorXd& jvel);
    Eigen::VectorXd CalTorqueInertia(const Eigen::VectorXd &jpos, const Eigen::VectorXd &jvel,const Eigen::VectorXd &jacc);
   

    JointStateEstimator estimator;
    bool initial_flag_rbdl = true;
    bool save_flag = true;
    int arm_type = 1;  
    DynamicsParams arm_dynamics_params;
    double L3 = 0.24; 
    double L6 = 0.182;
    double m3 = 0.383;   
    //double m6 = 0.201;
    double m6 = 0.0;
    double L3_max = 0.2410001;
    double L3_min = 0.21300001;
    double L6_max = 0.25100001;
    double L6_min = 0.15000001;
    double dither_phase_ = 0.0;

    const double FRICTION_DEATH_VEL[2][2] = { { 0.005, 0.5 },{ 0.001, 0.1 } };
 };