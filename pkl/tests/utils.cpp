#include "utils.hpp"

PKL::DualQuaternion vec2dq(const Vec& vec) {
    return PKL::DualQuaternion::PureTranslation(vec);
}

PKL::Homogeneous vec2hg(const Vec& vec) {
    return Homogeneous(Eigen::Transform<PKL::Scalar, 3, Eigen::Isometry>(
        Eigen::Translation<PKL::Scalar, 3>(vec)).matrix());
}

bool is_close(const Eigen::Matrix<PKL::Scalar, -1, -1>& mat1, const Eigen::Matrix<PKL::Scalar, -1, -1>& mat2) {
    static const PKL::Scalar eps = 1E-4;
    return (mat1 - mat2).norm() < eps;
}

bool is_close(const DualQuaternion& mat1, const DualQuaternion& mat2) {
    return is_close(mat1.to_twist_vec(), mat2.to_twist_vec());
}
