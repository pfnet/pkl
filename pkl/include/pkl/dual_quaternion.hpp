#ifndef PKL_DUAL_QUATERNION_HPP_
#define PKL_DUAL_QUATERNION_HPP_

#include <Eigen/Geometry>
#include <cmath>
#include "pkl/typedef.h"

namespace PKL {
class DualQuaternion {
    typedef Eigen::Quaternion<Scalar> Quaternion;

 public:
    DualQuaternion() {
        // identity: xyzw
        _rot.coeffs() << 0.0, 0.0, 0.0, 1.0;
        _trans.coeffs() << 0.0, 0.0, 0.0, 0.0;
    }

    DualQuaternion(const Quaternion &rot, const Quaternion& trans):
        _rot(rot), _trans(trans)
    {
    }

    DualQuaternion(const Vec& axis, const DualQuaternion& offset, Scalar theta) {
        // efficient computation of pure translation * general dual quaternion
        Quaternion rot(RotAroundAxis(axis, theta));
        _rot = rot * offset._rot;
        _trans = rot * offset._trans;
    }

    Eigen::Transform<Scalar, 3, Eigen::Isometry> to_homogeneous() const {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> ret;
        ret.linear() = _rot.toRotationMatrix();
        ret.translation() = 2.0 * (_trans * _rot.conjugate()).coeffs().head<3>();
        return ret;
    }
    inline const Quaternion& rot() const {
        return _rot;
    }
    inline const Quaternion& trans() const {
        return _trans;
    }
    inline void shift_translation(const DualQuaternion& other) {
        _trans.coeffs() -= (other._trans * other._rot.conjugate() * _rot).coeffs();
    }
    inline DualQuaternion inverse() const {
        return DualQuaternion(_rot.conjugate(), _trans.conjugate());
    }
    inline TwistVec to_twist_vec() const {
        TwistVec ret;
        ret.head<3>() = _trans.coeffs().head<3>();
        ret.tail<3>() = _rot.coeffs().head<3>();
        return ret;
    }
    inline static DualQuaternion product(const DualQuaternion& lhs, const DualQuaternion& rhs) {
        return DualQuaternion(
            lhs._rot * rhs._rot,
            Quaternion((lhs._rot * rhs._trans).coeffs() + (lhs._trans * rhs._rot).coeffs()));
    }
    // just for compatilibily with eigen interface
    inline static DualQuaternion Identity() {
        return DualQuaternion();
    }
    inline static Quaternion RotAroundAxis(const Eigen::Matrix<Scalar, 3, 1>& axis, Scalar theta) {
        return Quaternion(Eigen::AngleAxis<Scalar>(theta, axis));
    }
    inline static DualQuaternion PureRotation(const Eigen::Matrix<Scalar, 3, 1>& axis, Scalar theta) {
        DualQuaternion ret;
        ret._rot = RotAroundAxis(axis, theta);
        return ret;
    }
    inline static DualQuaternion PureTranslation(const Eigen::Matrix<Scalar, 3, 1>& axis) {
        DualQuaternion ret;
        ret._trans.coeffs().head<3>() = 0.5 * axis;
        return ret;
    }
    friend DualQuaternion operator*(const DualQuaternion& lhs, const DualQuaternion& rhs);
 private:
    Quaternion _rot;
    Quaternion _trans;
};
inline DualQuaternion operator*(const DualQuaternion& lhs, const DualQuaternion& rhs) {
    return DualQuaternion::product(lhs, rhs);
}
};  // namespace PKL
#endif  // PKL_DUAL_QUATERNION_HPP_
