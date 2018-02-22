#ifndef PKL_CHAIN_HPP_
#define PKL_CHAIN_HPP_

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <vector>
#include "pkl/typedef.h"
#include "pkl/joint.hpp"
#include "pkl/dual_quaternion.hpp"
#include "pkl/homogeneous.hpp"

namespace PKL {
PKL::Scalar singularity_safe(PKL::Scalar s);

#define SDIM (NUM_JOINTS < 6 ? NUM_JOINTS : 6)
template<int NUM_JOINTS>
inline Angles inverse_vel_template(const Jacobian& jacobian, const TwistVec& desired_twist) {
    // xi = J d_theta
    // J = USV^T
    // return d_theta = J^(-1) xi = V S^(-1) U^T xi
    // This is the current bottleneck
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, 6, NUM_JOINTS> > svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // U : 6 X 6
    // S : SDIM=min(6, NUM_JOINTS) X 1
    // V : NUM_JOINTS X NUM_JOINTS
    // We could also specialize these operations with known SDIM, but it's not as big of an improvement
    // so we will just use dynamic sizes
    Eigen::Matrix<Scalar, SDIM, 1> Uxi = (svd.matrixU().transpose() * desired_twist).template block<SDIM, 1>(0, 0);
    Eigen::Array<Scalar, SDIM, 1> ss = svd.singularValues();
    ss = ss.unaryExpr(std::ptr_fun(singularity_safe));
    Eigen::Matrix<Scalar, SDIM, 1> SUxi = ss.cwiseProduct(Uxi.array());

    return svd.matrixV().template block<NUM_JOINTS, SDIM>(0, 0) * SUxi;
}
#undef SDIM

template<>
inline Angles inverse_vel_template<-1>(const Jacobian& jacobian, const TwistVec& desired_twist)  {
    const int num_joints = jacobian.cols();
    const int sdim = std::min(num_joints, 6);
    // This is the current bottleneck
    Eigen::JacobiSVD<Jacobian> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<Scalar, -1, 1> Uxi = (svd.matrixU().transpose() * desired_twist).block(0, 0, sdim, 1);
    Eigen::Array<Scalar, -1, 1> ss = svd.singularValues();
    ss = ss.unaryExpr(std::ptr_fun(singularity_safe));
    Eigen::Matrix<Scalar, -1, 1> SUxi = ss.cwiseProduct(Uxi.array());

    return svd.matrixV().block(0, 0, num_joints, sdim) * SUxi;
}

template<class TF_TYPE>
class Chain {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Chain(): _start_frame(TF_TYPE::Identity()), _num_joints(0) {}
    explicit Chain(const TF_TYPE& start_frame): _start_frame(start_frame), _num_joints(0) {}

    // number of all joints including rigid ones
    inline size_t num_joints() const { return _joints.size(); }
    // number of non-rigid joints
    inline size_t num_non_rigid_joints() const { return _num_joints; }

    inline TF_TYPE forward(const Angles& thetas) const;
    inline Jacobian jacobian(const Angles& thetas) const;
    inline Angles inverse_vel(const Angles& current_angles, const TwistVec& desired_twist) const;

    template<template<JointType> class JOINT_IMPL>
    inline void add_joint(JointType joint_type, const Vec& axis, const TF_TYPE& offset);

    template<template<JointType> class JOINT_IMPL>
    inline void add_rigid_joint(const TF_TYPE& offset);

    // default template argument for template template parameter is not allowed (before c++11)
    // so we provide a non-template definition to use default implementation
    inline void add_joint(JointType joint_type, const Vec& axis, const TF_TYPE& offset);
    inline void add_rigid_joint(const TF_TYPE& offset);

 protected:
    static Eigen::Matrix<Scalar, 6, 1> adjoint(const TF_TYPE& tf, const Joint<TF_TYPE>& joint);
    static TF_TYPE make_inverse(const TF_TYPE& tf);

    int _num_joints;
    std::vector<Joint<TF_TYPE> > _joints;
    const TF_TYPE _start_frame;
};
}  // namespace PKL

#include "chain.inl"
#endif  // PKL_CHAIN_HPP_
