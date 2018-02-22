#ifndef PKL_TEST_UTILS_HPP_
#define PKL_TEST_UTILS_HPP_

#include "pkl/chain.hpp"
#include "pkl/dual_quaternion.hpp"
#include "pkl/homogeneous.hpp"
#include "pkl/matrix_rp.hpp"

using PKL::DualQuaternion;
using PKL::Homogeneous;
using PKL::MatrixRP;
using PKL::Vec;

const int num_joints = 7;

PKL::DualQuaternion vec2dq(const Vec& vec);
PKL::Homogeneous vec2hg(const Vec& vec);
bool is_close(
    const Eigen::Matrix<PKL::Scalar, -1, -1>& mat1,
    const Eigen::Matrix<PKL::Scalar, -1, -1>& mat2);

bool is_close(
    const DualQuaternion& mat1,
    const DualQuaternion& mat2);

template<class TF_TYPE>
static PKL::Chain<TF_TYPE> make_puma_chain() {
    static const PKL::Scalar d3 = 0.31;
    static const PKL::Scalar d5 = 0.31;
    static const PKL::Scalar d7 = 0.16;
    const auto ey = Vec(0.0, 1.0, 0.0);
    const auto ez = Vec(0.0, 0.0, 1.0);
    const auto zero_offset = TF_TYPE::PureTranslation(Vec::Zero());
    const auto d3_tf = TF_TYPE::PureTranslation(d3 * ez);
    const auto d5_tf = TF_TYPE::PureTranslation(d5 * ez);
    const auto d7_tf = TF_TYPE::PureTranslation(d7 * ez);

    PKL::Chain<TF_TYPE> ret = PKL::Chain<TF_TYPE>();
    ret.add_joint(PKL::JointTypeRev, ez, zero_offset);
    ret.add_joint(PKL::JointTypeRev, ey, zero_offset);
    ret.add_joint(PKL::JointTypeRev, ez, d3_tf);
    ret.add_joint(PKL::JointTypeRev, ey, zero_offset);
    ret.add_joint(PKL::JointTypeRev, ez, d5_tf);
    ret.add_joint(PKL::JointTypeRev, ey, zero_offset);
    ret.add_joint(PKL::JointTypeRev, ez, d7_tf);

    return ret;
}

#endif  // PKL_TEST_UTILS_HPP_
