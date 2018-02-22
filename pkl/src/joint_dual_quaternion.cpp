#include "pkl/joint.hpp"
#include "pkl/dual_quaternion.hpp"
#include "pkl/joint_default_impl.hpp"

namespace PKL {
template<>
template<>
DualQuaternion DefaultImpl<DualQuaternion>::JointImpl<JointTypeRev>::pose(
    Scalar theta, const Vec& axis, const DualQuaternion& offset)
{
    return DualQuaternion(axis, offset, theta);
}

template<>
template<>
DualQuaternion DefaultImpl<DualQuaternion>::JointImpl<JointTypePrism>::pose(
    Scalar theta, const Eigen::Matrix<Scalar, 3, 1>& axis, const DualQuaternion& offset)
{
    return DualQuaternion::PureTranslation(theta * axis) * offset;
}

template<>
template<>
DualQuaternion DefaultImpl<DualQuaternion>::JointImpl<JointTypeRigid>::pose(
    Scalar, const Eigen::Matrix<Scalar, 3, 1>&, const DualQuaternion& offset)
{
    return offset;
}

template<>
template<>
TwistVec DefaultImpl<DualQuaternion>::JointImpl<JointTypeRev>::adjoint(const DualQuaternion& tf, const Vec& twist) {
    // R \omega, p X R \omega
    const Vec omega(tf.rot().toRotationMatrix() * twist);
    const Vec v(2.0 * (tf.trans() * tf.rot().conjugate()).coeffs().head<3>().cross(omega));
    TwistVec ret;
    ret << v, omega;
    return ret;
}

template<>
template<>
TwistVec DefaultImpl<DualQuaternion>::JointImpl<JointTypePrism>::adjoint(const DualQuaternion& tf, const Vec& twist) {
    // R v
    TwistVec ret;
    ret << (tf.rot().toRotationMatrix() * twist), 0.0, 0.0, 0.0;
    return ret;
}
}  // namespace PKL
