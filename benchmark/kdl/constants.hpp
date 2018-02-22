#ifndef BENCHMARK_KDL_BENCHMARK_CONSTANTS_HPP_
#define BENCHMARK_KDL_BENCHMARK_CONSTANTS_HPP_

#include <kdl/chain.hpp>
#include <pkl/chain.hpp>

namespace BENCHMARK_CONSTANTS {
using PKL::Vec;
const PKL::Scalar d3 = 0.31;
const PKL::Scalar d5 = 0.31;
const PKL::Scalar d7 = 0.16;
const int num_joints = 7;
KDL::Chain make_puma_kdl_chain();
template<class TF_TYPE> PKL::Chain<TF_TYPE> make_puma_pkl_chain();

template<class TF_TYPE>
static TF_TYPE make_offset(PKL::Scalar x, PKL::Scalar y, PKL::Scalar z) {
    return TF_TYPE::PureTranslation({x, y, z});
}

template<class TF_TYPE>
PKL::Chain<TF_TYPE> make_puma_pkl_chain() {
    PKL::Chain<TF_TYPE> chain;
    chain.add_joint(PKL::JointTypeRev, Vec(0.0, 0.0, 1.0), make_offset<TF_TYPE>(0.0, 0.0, 0.0));
    chain.add_joint(PKL::JointTypeRev, Vec(0.0, 1.0, 0.0), make_offset<TF_TYPE>(0.0, 0.0, 0.0));
    chain.add_joint(PKL::JointTypeRev, Vec(0.0, 0.0, 1.0), make_offset<TF_TYPE>(0.0, 0.0, d3));
    chain.add_joint(PKL::JointTypeRev, Vec(0.0, 1.0, 0.0), make_offset<TF_TYPE>(0.0, 0.0, 0.0));
    chain.add_joint(PKL::JointTypeRev, Vec(0.0, 0.0, 1.0), make_offset<TF_TYPE>(0.0, 0.0, d5));
    chain.add_joint(PKL::JointTypeRev, Vec(0.0, 1.0, 0.0), make_offset<TF_TYPE>(0.0, 0.0, 0.0));
    chain.add_joint(PKL::JointTypeRev, Vec(0.0, 0.0, 1.0), make_offset<TF_TYPE>(0.0, 0.0, d7));
    return chain;
}
};  // namespace BENCHMARK_CONSTANTS
#endif  // BENCHMARK_KDL_BENCHMARK_CONSTANTS_HPP_
