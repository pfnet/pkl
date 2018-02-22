#include <benchmark/benchmark.h>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <pkl/chain.hpp>
#include "constants.hpp"
#include <pkl/dual_quaternion.hpp>
#include <pkl/homogeneous.hpp>
#include <pkl/matrix_rp.hpp>

using PKL::DualQuaternion;
using PKL::Homogeneous;
using PKL::MatrixRP;

typedef Eigen::Matrix<PKL::Scalar, 6, 7> Jacobian;
static const int num_joints = BENCHMARK_CONSTANTS::num_joints;

template<class TF_TYPE>
static Jacobian pkl_jacobian(const PKL::Chain<TF_TYPE>& chain, const PKL::Angles& theta) {
    auto ret = chain.jacobian(theta);
    return ret;
}

static KDL::Jacobian kdl_jacobian(KDL::ChainJntToJacSolver& solver, const KDL::JntArray& jntArray) {
    KDL::Jacobian jac(7);
    solver.JntToJac(jntArray, jac, num_joints);
    return jac;
}

void BM_PKL_JACOBIAN_DQ(benchmark::State& state) {
    auto chain(BENCHMARK_CONSTANTS::make_puma_pkl_chain<DualQuaternion>());
    PKL::Angles thetas = Eigen::Matrix<PKL::Scalar, num_joints, 1>::Zero();

    for (auto _ : state) {
        Jacobian pklj = pkl_jacobian<DualQuaternion>(chain, thetas);
        benchmark::DoNotOptimize(pklj);
    }
}

void BM_PKL_JACOBIAN_HG(benchmark::State& state) {
    const auto chain = BENCHMARK_CONSTANTS::make_puma_pkl_chain<Homogeneous>();
    PKL::Angles thetas = Eigen::Matrix<PKL::Scalar, num_joints, 1>::Zero();

    for (auto _ : state) {
        Jacobian pklj = pkl_jacobian<Homogeneous>(chain, thetas);
        benchmark::DoNotOptimize(pklj);
    }
}

void BM_PKL_JACOBIAN_RP(benchmark::State& state) {
    const auto chain = BENCHMARK_CONSTANTS::make_puma_pkl_chain<MatrixRP>();
    PKL::Angles thetas = Eigen::Matrix<PKL::Scalar, num_joints, 1>::Zero();

    for (auto _ : state) {
        Jacobian pklj = pkl_jacobian<MatrixRP>(chain, thetas);
        benchmark::DoNotOptimize(pklj);
    }
}

void BM_KDL_JACOBIAN(benchmark::State& state) {
    auto chain = BENCHMARK_CONSTANTS::make_puma_kdl_chain();
    auto solver = KDL::ChainJntToJacSolver(chain);
    KDL::JntArray jntArray(num_joints);
    KDL::Jacobian kdlj(7);

    for (auto _ : state) {
        kdlj = kdl_jacobian(solver, jntArray);
        benchmark::DoNotOptimize(kdlj);
    }
}
