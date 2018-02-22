#include <benchmark/benchmark.h>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <pkl/chain.hpp>
#include <pkl/dual_quaternion.hpp>
#include <pkl/homogeneous.hpp>
#include "constants.hpp"

using PKL::DualQuaternion;
using PKL::Homogeneous;
using PKL::MatrixRP;

static const int num_joints = BENCHMARK_CONSTANTS::num_joints;

static const PKL::Angles theta = Eigen::Matrix<PKL::Scalar, num_joints, 1>::Random();
static const PKL::TwistVec desired_twist = PKL::TwistVec::Random();

template<class TF_TYPE>
Eigen::Matrix<PKL::Scalar, num_joints, 1> pkl_inverse(
    const PKL::Chain<TF_TYPE>& chain, const PKL::Angles& theta, const PKL::TwistVec& desired_twist)
{
    auto ret = chain.inverse_vel(theta, desired_twist);
    return ret;
}

KDL::JntArray kdl_inverse(KDL::ChainIkSolverVel_pinv& solver, const KDL::JntArray& jntArray, const KDL::Twist& v_in) {
    KDL::JntArray ret(num_joints);
    solver.CartToJnt(jntArray, v_in, ret);
    return ret;
}

void BM_PKL_INVERSE_DQ(benchmark::State& state) {
    const auto chain = BENCHMARK_CONSTANTS::make_puma_pkl_chain<DualQuaternion>();

    for (auto _ : state) {
        pkl_inverse(chain, theta, desired_twist);
    }
}

void BM_PKL_INVERSE_HG(benchmark::State& state) {
    const auto chain = BENCHMARK_CONSTANTS::make_puma_pkl_chain<Homogeneous>();

    for (auto _ : state) {
        pkl_inverse(chain, theta, desired_twist);
    }
}

void BM_PKL_INVERSE_RP(benchmark::State& state) {
    const auto chain = BENCHMARK_CONSTANTS::make_puma_pkl_chain<MatrixRP>();

    for (auto _ : state) {
        pkl_inverse(chain, theta, desired_twist);
    }
}

void BM_KDL_INVERSE(benchmark::State& state) {
    auto chain = BENCHMARK_CONSTANTS::make_puma_kdl_chain();
    auto solver = KDL::ChainIkSolverVel_pinv(chain);
    KDL::JntArray jntArray(num_joints);
    KDL::JntArray result(num_joints);

    // for fair comparison, copy PKL values
    KDL::Twist kdl_desired_twist(
        KDL::Vector(desired_twist(0), desired_twist(1), desired_twist(2)),
        KDL::Vector(desired_twist(3), desired_twist(4), desired_twist(5)));
    for (int i = 0; i < num_joints; ++i) {
        jntArray(i) = theta(i);
    }

    for (auto _ : state) {
        result = kdl_inverse(solver, jntArray, kdl_desired_twist);
        benchmark::DoNotOptimize(result);
    }
}
