#pragma once
#ifndef BIQUADFILTER_H
#define BIQUADFILTER_H
#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <stdexcept>
#include <fstream>


#define DTOF 7
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ----------- BiQuad 滤波器类（单通道）-----------
class BiQuadFilter 
{
private:
    double b0_, b1_, b2_;
    double a1_, a2_;
    double x1_, x2_; // 输入历史
    double y1_, y2_; // 输出历史

public:
    BiQuadFilter() { reset(); }

    void setLowPass(double sampleRate, double cutoffFreq, double Q = 0.7071) {
        double omega = 2.0 * M_PI * cutoffFreq / sampleRate;
        double cos_omega = std::cos(omega);
        double sin_omega = std::sin(omega);
        double alpha = sin_omega / (2.0 * Q);

        double a0 = 1.0 + alpha;
        b0_ = (1.0 - cos_omega) / (2.0 * a0);
        b1_ = (1.0 - cos_omega) / a0;
        b2_ = (1.0 - cos_omega) / (2.0 * a0);
        a1_ = -2.0 * cos_omega / a0;
        a2_ = (1.0 - alpha) / a0;


        // std::cout << "a0\n" <<a0 << "\n";
        // std::cout << "b0_\n" <<b0_ << "\n";
        // std::cout << "b1_\n" <<b1_ << "\n";
        // std::cout << "b2_\n" <<b2_ << "\n";
        // std::cout << "a1_\n" <<a1_ << "\n";
        // std::cout << "a2_\n" <<a2_ << "\n";
        // std::cout << "sampleRate\n" <<sampleRate << "\n";
        // std::cout << "cutoffFreq\n" <<cutoffFreq << "\n";


        // Direct Form II Transposed 不需要显式除 a0（已包含）
    }

    double process(double x) 
    {
        double y = b0_ * x + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
        x2_ = x1_; x1_ = x;
        y2_ = y1_; y1_ = y;
   
        // std::cout << "x\n" << x  << "\n";
        // std::cout << "b0_\n" <<b0_ << "\n";
        // std::cout << "b1_\n" <<b1_ << "\n";
        // std::cout << "b2_\n" <<b2_ << "\n";
        // std::cout << "a1_\n" <<a1_ << "\n";
        // std::cout << "a2_\n" <<a2_ << "\n";
        // std::cout << "y\n" << y << "\n";

        return y;
    }

    void setInitialPos(double x0) 
    {
        x1_ = x2_ = x0;
        y1_ = y2_ = x0;  // 假设稳态输出 = 输入

    }

    void reset() 
    {
        b0_ = b1_ = b2_ = a1_ = a2_ = 0.0;
        x1_ = x2_ = y1_ = y2_ = 0.0;
        //std::cout << "BiQuadFilter reset\n";
    }
};



// ----------- 关节状态估计器（多通道）-----------
class JointStateEstimator 
{
private:
    const int n_joints_;
    const double dt_; // 固定采样时间（秒）

    Eigen::VectorXd q_prev_;
    Eigen::VectorXd qd_filtered_prev_;

    // 每个关节一个速度滤波器 + 一个加速度滤波器
    std::vector<BiQuadFilter> pos_filters_;
    std::vector<BiQuadFilter> vel_filters_;
    std::vector<BiQuadFilter> acc_filters_;

public:
    JointStateEstimator(int n_joints, //const Eigen::VectorXd& q_initial,
                        double control_freq_hz,
                        double pos_cutoff_hz = 10.0,
                        double vel_cutoff_hz = 10.0,
                        double acc_cutoff_hz = 10.0)
            : n_joints_(n_joints),
            // q_prev_(q_initial),
            dt_(1.0 / control_freq_hz),
            q_prev_(Eigen::VectorXd::Zero(n_joints)),
            qd_filtered_prev_(Eigen::VectorXd::Zero(n_joints)),
            vel_filters_(n_joints),
            pos_filters_(n_joints),
            acc_filters_(n_joints) {

        // 初始化所有滤波器
        for (int i = 0; i < n_joints_; ++i) {
            pos_filters_[i].setLowPass(control_freq_hz, pos_cutoff_hz);
            vel_filters_[i].setLowPass(control_freq_hz, vel_cutoff_hz);
            acc_filters_[i].setLowPass(control_freq_hz, acc_cutoff_hz);
        }
    }

	void InitialJoint(const Eigen::VectorXd& q_initial)
	{
    	q_prev_ = q_initial;

         for (int i = 0; i < n_joints_; ++i) 
         {
            pos_filters_[i].setInitialPos(q_initial(i));           
         }

        //std::cout << "Initial\n"<< q_prev_(1)  << "\n";
	}



    void update(Eigen::VectorXd& q_curr,
                Eigen::VectorXd& qd_out,
                Eigen::VectorXd& qdd_out) 
    {
       // std::cout << "before q_curr\n"<< q_curr(1)  << "\n";

        q_curr.resize(n_joints_);

        for (int i = 0; i < n_joints_; ++i) 
        {
            q_curr(i) = pos_filters_[i].process(q_curr(i));
        }

        //std::cout << "after q_curr\n"<< q_curr(1)  << "\n";

        // 1. 计算原始速度（前向差分）
        Eigen::VectorXd qd_raw = (q_curr - q_prev_) / dt_;

        // 2. 对速度滤波
        qd_out.resize(n_joints_);
        for (int i = 0; i < n_joints_; ++i) {
            qd_out(i) = vel_filters_[i].process(qd_raw(i));
        }

        // std::cout << "q_curr\n"<< q_curr(0)  << "\n";
        // std::cout << "q_prev_\n"<< q_prev_(0)  << "\n";       
        // std::cout << "qd_raw\n"<< qd_raw (0) << "\n";
        // std::cout << "qd_out\n"<< qd_out(0)  << "\n";

        // std::cout << "q_curr\n"<< q_curr  << "\n";
        // std::cout << "q_prev_\n"<< q_prev_  << "\n";       
        // std::cout << "qd_raw\n"<< qd_raw  << "\n";
        // std::cout << "qd_out\n"<< qd_out  << "\n";
        // 3. 用滤波后的速度计算原始加速度
        Eigen::VectorXd qdd_raw = (qd_out - qd_filtered_prev_) / dt_;


        // 4. 对加速度滤波
        qdd_out.resize(n_joints_);
        for (int i = 0; i < n_joints_; ++i) 
        {
            qdd_out(i) = acc_filters_[i].process(qdd_raw(i));
        }

        // std::cout << "qd_out\n"<< qd_out(0)  << "\n";
        // std::cout << "qd_filtered_prev_\n"<< qd_filtered_prev_(0)  << "\n";       
        // std::cout << "qdd_raw\n"<< qdd_raw (0) << "\n";
        // std::cout << "qdd_out\n"<< qdd_out(0)  << "\n";
    
        // 5. 更新历史状态
        q_prev_ = q_curr;
        qd_filtered_prev_ = qd_out;
    }

    
    void reset() {
        q_prev_.setZero();
        qd_filtered_prev_.setZero();
        for (auto& f : vel_filters_) f.reset();
        for (auto& f : acc_filters_) f.reset();
    }
};
#endif

