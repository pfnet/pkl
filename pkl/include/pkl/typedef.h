#pragma once
#include <Eigen/Geometry>

namespace PKL {
typedef float Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vec;
typedef Eigen::Matrix<Scalar, 6, 1> TwistVec;
typedef Eigen::Matrix<Scalar, 6, -1> Jacobian;
typedef Eigen::Matrix<Scalar, -1, 1> Angles;
}  // namespace PKL
