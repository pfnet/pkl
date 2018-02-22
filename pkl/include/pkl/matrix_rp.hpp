#ifndef PKL_MATRIX_RP_HPP_
#define PKL_MATRIX_RP_HPP_

#include <Eigen/Core>
#include "pkl/typedef.h"
#include "joint.hpp"

namespace PKL {
class MatrixRP {
    typedef Eigen::Matrix<Scalar, 3, 3> Mat;

 public:
    MatrixRP():
        _rot(Mat::Identity()),
        _trans(Vec::Zero())
    {
    }
    MatrixRP(const Mat& rot, const Vec& trans):
        _rot(rot),
        _trans(trans)
    {
    }
    inline void shift_translation(const MatrixRP& other) {
        _trans -= other._trans;
    }
    inline MatrixRP inverse() const {
        Mat tf_inv(_rot.transpose());
        return MatrixRP(tf_inv, -tf_inv * _trans);
    }
    inline Mat get_rot() const {
        return _rot;
    }
    inline Vec get_trans() const {
        return _trans;
    }
    inline static MatrixRP Identity() {
        return MatrixRP();
    }
    inline static MatrixRP PureRotation(const Eigen::Matrix<Scalar, 3, 1>& axis, Scalar theta) {
        return MatrixRP(
            Eigen::AngleAxis<Scalar>(theta, axis).toRotationMatrix(),
            Vec::Zero());
    }
    inline static MatrixRP PureTranslation(const Vec& axis) {
        return MatrixRP(
            Mat::Identity(),
            axis);
    }

    friend MatrixRP operator*(const MatrixRP& lhs, const MatrixRP& rhs);

 private:
    Mat _rot;
    Vec _trans;
};

inline MatrixRP operator*(const MatrixRP& lhs, const MatrixRP& rhs) {
    return MatrixRP(
        lhs._rot * rhs._rot,
        lhs._trans + lhs._rot * rhs._trans);
}
};  // namespace PKL
#endif  // PKL_MATRIX_RP_HPP_
