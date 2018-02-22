#ifndef PKL_CHAIN_INL_
#define PKL_CHAIN_INL_

#include <cassert>
#include "pkl/joint_default_impl.hpp"

namespace PKL {
template<class TF_TYPE>
inline TF_TYPE Chain<TF_TYPE>::forward(const Angles& thetas) const {
    assert(thetas.rows() == _num_joints);
    TF_TYPE ret = _start_frame;
    size_t non_rigid_cnt = 0;
    for (size_t idx = 0; idx < _joints.size(); ++idx) {
        if (_joints[idx].is_rigid())
            ret = ret * _joints[idx].pose(0.0);
        else
            ret = ret * _joints[idx].pose(thetas(non_rigid_cnt++));
    }
    return ret;
}

template<class TF_TYPE>
inline Jacobian Chain<TF_TYPE>::jacobian(const Angles& thetas) const {
    assert(thetas.rows() == _num_joints);
    TF_TYPE T = _start_frame;
    std::vector<TF_TYPE> tfs(_num_joints);
    // perform forward kinematics and save the each joint frames in tfs
    int non_rigid_cnt = 0;
    for (size_t idx = 0; idx < _joints.size(); ++idx) {
        Scalar theta_for_joint = 0;
        if (!_joints[idx].is_rigid()) {
            theta_for_joint = thetas(non_rigid_cnt);
            tfs[non_rigid_cnt] = T;
            ++non_rigid_cnt;
        }
        T = T * _joints[idx].pose(theta_for_joint);
    }
    Jacobian ret(6, _num_joints);
    non_rigid_cnt = 0;

    for (size_t idx = 0; idx < _joints.size(); ++idx) {
        if (!_joints[idx].is_rigid()) {
            tfs[non_rigid_cnt].shift_translation(T);
            ret.col(non_rigid_cnt) = _joints[idx].adjoint(tfs[non_rigid_cnt]);
            ++non_rigid_cnt;
        }
    }
    return ret;
}

template<class TF_TYPE>
inline Angles Chain<TF_TYPE>::inverse_vel(const Angles& current_angles, const TwistVec& desired_twist) const {
    Jacobian jac = jacobian(current_angles);

    // SVD is significantly faster (X1.5) if the jacobian matrix size is known at compile time.
    // Therefore we declare all jacobian with practical number of joints and select one of those at run time.
    switch (_num_joints) {
        case 5:
            return inverse_vel_template<5>(jac, desired_twist);
        case 6:
            return inverse_vel_template<6>(jac, desired_twist);
        case 7:
            return inverse_vel_template<7>(jac, desired_twist);
        case 8:
            return inverse_vel_template<8>(jac, desired_twist);
        default:
            // -1 will invoke dynamic size
            return inverse_vel_template<-1>(jac, desired_twist);
    }
}

template<class TF_TYPE>
template<template<JointType> class JOINT_IMPL>
inline void Chain<TF_TYPE>::add_joint(
    JointType joint_type,
    const Vec& axis,
    const TF_TYPE& offset)
{
    // can't use constexpr in c++98
    switch (joint_type) {
        case JointTypeRev:
            _joints.push_back(Joint<TF_TYPE>(axis, offset, JOINT_IMPL<JointTypeRev>()));
            ++_num_joints;
            return;
        case JointTypePrism:
            _joints.push_back(Joint<TF_TYPE>(axis, offset, JOINT_IMPL<JointTypePrism>()));
            ++_num_joints;
            return;
        case JointTypeRigid:
            assert("axis provided for rigid joint");
    }
}

template<class TF_TYPE>
template<template<JointType> class JOINT_IMPL>
inline void Chain<TF_TYPE>::add_rigid_joint(const TF_TYPE& offset) {
    _joints.push_back(Joint<TF_TYPE>(offset, JOINT_IMPL<JointTypeRigid>()));
}

template<class TF_TYPE>
inline void Chain<TF_TYPE>::add_joint(
    JointType joint_type,
    const Vec& axis,
    const TF_TYPE& offset)
{
    add_joint<DefaultImpl<TF_TYPE>::template JointImpl>(joint_type, axis, offset);
}

template<class TF_TYPE>
inline void Chain<TF_TYPE>::add_rigid_joint(const TF_TYPE& offset) {
    add_rigid_joint<DefaultImpl<TF_TYPE>::template JointImpl>(offset);
}

template<class TF_TYPE>
inline TF_TYPE Chain<TF_TYPE>::make_inverse(const TF_TYPE& tf) {
    return tf.inverse();
}
};  // namespace PKL

#endif  // PKL_CHAIN_INL_
