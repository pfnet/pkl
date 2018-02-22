#include "pkl/typedef.h"
#include "pkl/joint.hpp"
#include "pkl/matrix_rp.hpp"
#include "pkl/joint_default_impl.hpp"

namespace PKL {
template<>
template<>
MatrixRP DefaultImpl<MatrixRP>::JointImpl<JointTypeRev>::pose(
    Scalar theta,
    const Vec& axis,
    const MatrixRP& offset)
{
    return MatrixRP::PureRotation(axis, theta) * offset;
}

template<>
template<>
MatrixRP DefaultImpl<MatrixRP>::JointImpl<JointTypePrism>::pose(
    Scalar theta, const Vec& axis, const MatrixRP& offset)
{
    return MatrixRP::PureTranslation(theta * axis) * offset;
}

template<>
template<>
MatrixRP DefaultImpl<MatrixRP>::JointImpl<JointTypeRigid>::pose(
    Scalar, const Vec&, const MatrixRP& offset)
{
    return offset;
}

template<>
template<>
TwistVec DefaultImpl<MatrixRP>::JointImpl<JointTypeRev>::adjoint(
    const MatrixRP& tf,
    const Vec& twist)
{
    const Vec omega(tf.get_rot() * twist);
    const Vec v(tf.get_trans().cross(omega));
    TwistVec ret;
    ret << v, omega;
    return ret;
}

template<>
template<>
TwistVec DefaultImpl<MatrixRP>::JointImpl<JointTypePrism>::adjoint(
    const MatrixRP& tf,
    const Vec& twist)
{
    TwistVec ret;
    ret << (tf.get_rot() * twist), 0.0, 0.0, 0.0;
    return ret;
}
}  // namespace PKL
