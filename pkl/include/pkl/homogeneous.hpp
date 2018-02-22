#ifndef PKL_HOMOGENEOUS_HPP_
#define PKL_HOMOGENEOUS_HPP_

#include <Eigen/Core>
#include "pkl/typedef.h"

namespace PKL {
class Homogeneous {
    typedef Eigen::Matrix<Scalar, 4, 4> Mat;

 public:
    Homogeneous(): _mat(Mat::Identity()) {
    }
    explicit Homogeneous(const Mat& mat): _mat(mat) {
    }

    inline void shift_translation(const Homogeneous& other) {
        _mat.block<3, 1>(0, 3) -= other._mat.block<3, 1>(0, 3);
    }
    inline const Mat& mat() const { return _mat; }
    inline Homogeneous inverse() const {
        Mat tf_inv;
        tf_inv.block<3, 3>(0, 0) = _mat.block<3, 3>(0, 0).transpose();
        tf_inv.block<3, 1>(0, 3) = -tf_inv.block<3, 3>(0, 0) * _mat.block<3, 1>(0, 3);
        tf_inv.block<1, 3>(3, 0).setZero();
        tf_inv(3, 3) = 1;
        return Homogeneous(tf_inv);
    }
    inline static Homogeneous Identity() {
        return Homogeneous();
    }
    inline static Homogeneous PureRotation(const Vec& axis, Scalar theta) {
        return Homogeneous(Eigen::Transform<Scalar, 3, Eigen::Isometry>(
            Eigen::AngleAxis<Scalar>(theta, axis)).matrix());
    }
    inline static Homogeneous PureTranslation(const Vec& axis) {
        return Homogeneous(Eigen::Transform<Scalar, 3, Eigen::Isometry>(
            Eigen::Translation<Scalar, 3>(axis)).matrix());
    }
    friend Homogeneous operator*(const Homogeneous& lhs, const Homogeneous& rhs);

 private:
    Mat _mat;
};
inline Homogeneous operator*(const Homogeneous& lhs, const Homogeneous& rhs) {
    return Homogeneous(lhs._mat * rhs._mat);
}
};  // namespace PKL
#endif  // PKL_HOMOGENEOUS_HPP_
