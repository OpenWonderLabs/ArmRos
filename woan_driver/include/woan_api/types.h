#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>        // std::isnan, std::isinf, std::fabs
#include <iostream>
#include <sstream>
#include <string>

namespace woan {

// ==========================
// Type aliases
// ==========================
using JointVector6f = Eigen::Matrix<float, 6, 1>;
using JointVector7f = Eigen::Matrix<float, 7, 1>;
using RotMatrix     = Eigen::Matrix3d;
using Point         = Eigen::Vector3d;
using Euler         = Eigen::Vector3d;
using Quaternion    = Eigen::Quaterniond;

// ==========================
// Eigen → string helper
// ==========================
template <typename Derived>
inline std::string e2s(const Eigen::MatrixBase<Derived>& mat)
{
    std::ostringstream oss;
    oss << mat.transpose();
    return oss.str();
}

// ==========================
// Math constants & macros
// ==========================
constexpr float DEG2RAD(float x) noexcept { return x * 0.017453292519943295f; }
constexpr float RAD2DEG(float x) noexcept { return x * 57.29577951308232f; }

constexpr float N_PI   = 3.14159265358979323846f;
constexpr float N_PI_2 = 1.57079632679489661923f;
constexpr float N_2PI  = N_PI * 2.0f;

// ==========================
// wrap to [-pi, pi]
// ==========================
inline float wrapToPi(float a) noexcept
{
    if (std::isnan(a) || std::isinf(a))
    {
        return 0.0f;
    }

    while (std::fabs(a) > N_PI)
    {
        if (a > N_PI)
            a -= N_2PI;
        else
            a += N_2PI;
    }
    return a;
}

// ==========================
// Euler → Quaternion
// ==========================
inline Quaternion euler2Quaternion(const Euler& euler)
{
    Eigen::AngleAxisd rollAngle (euler(2), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(euler(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle  (euler(0), Eigen::Vector3d::UnitZ());
    return yawAngle * pitchAngle * rollAngle;
}

// ==========================
// Transform
// ==========================
using Translation = Point;

struct Transform
{
    Translation translation;
    Quaternion  rotation;
    Euler       euler;

    Transform()
        : translation(Translation::Zero())
        , rotation(RotMatrix::Identity())
        , euler(Euler::Zero())
    {}

    Transform(const Translation& t, const Quaternion& r)
        : translation(t), rotation(r)
    {
        euler = rotation.matrix().eulerAngles(2, 1, 0);
    }

    Transform(const Translation& t, const RotMatrix& r)
        : translation(t), rotation(r)
    {
        euler = rotation.matrix().eulerAngles(2, 1, 0);
    }

    Transform(const Translation& t, const Euler& e)
        : translation(t), euler(e)
    {
        rotation = euler2Quaternion(e);
    }

    Transform(const Transform&) = default;
    Transform& operator=(const Transform&) = default;

    friend std::ostream& operator<<(std::ostream& os, const Transform& obj)
    {
        os << obj.translation.transpose() << "; " << obj.euler.transpose();
        return os;
    }
};

// ==========================
// Posture
// ==========================
struct Posture
{
    Eigen::Vector3d     pose;
    Eigen::Quaterniond quat;

    Posture()
        : pose(Eigen::Vector3d::Zero())
        , quat(Eigen::Quaterniond::Identity())
    {}

    Posture(const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation)
        : pose(position)
        , quat(orientation)
    {}
};

} // namespace woan
