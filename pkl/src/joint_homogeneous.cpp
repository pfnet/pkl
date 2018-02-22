#include "pkl/joint.hpp"
#include "pkl/homogeneous.hpp"
#include "pkl/joint_default_impl.hpp"

namespace PKL {
template<>
template<>
Homogeneous DefaultImpl<Homogeneous>::JointImpl<JointTypeRev>::pose(
    Scalar theta,
    const Vec& axis,
    const Homogeneous& offset)
{
    return Homogeneous(
        Eigen::Transform<Scalar, 3, Eigen::Isometry>(Eigen::AngleAxis<Scalar>(theta, axis)).matrix() * offset.mat());
}

template<>
template<>
Homogeneous DefaultImpl<Homogeneous>::JointImpl<JointTypePrism>::pose(
    Scalar theta, const Vec& axis, const Homogeneous& offset)
{
    return Homogeneous(
        Eigen::Transform<Scalar, 3, Eigen::Isometry>(Eigen::Translation<Scalar, 3>(theta * axis)) * offset.mat());
}

template<>
template<>
Homogeneous DefaultImpl<Homogeneous>::JointImpl<JointTypeRigid>::pose(
    Scalar, const Vec&, const Homogeneous& offset)
{
    return offset;
}

template<>
template<>
TwistVec DefaultImpl<Homogeneous>::JointImpl<JointTypeRev>::adjoint(const Homogeneous& tf, const Vec& twist) {
    const Vec omega(tf.mat().block<3, 3>(0, 0) * twist);
    const Vec v(tf.mat().block<3, 1>(0, 3).cross(omega));
    TwistVec ret;
    ret << v, omega;
    return ret;
}

template<>
template<>
TwistVec DefaultImpl<Homogeneous>::JointImpl<JointTypePrism>::adjoint(const Homogeneous& tf, const Vec& twist) {
    TwistVec ret;
    ret << (tf.mat().block<3, 3>(0, 0) * twist), 0.0, 0.0, 0.0;
    return ret;
}
}  // namespace PKL
