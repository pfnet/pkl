#include <benchmark/benchmark.h>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <pkl/chain.hpp>
#include <pkl/dual_quaternion.hpp>
#include <pkl/homogeneous.hpp>
#include <pkl/matrix_rp.hpp>
#include "constants.hpp"

static const int num_joints = BENCHMARK_CONSTANTS::num_joints;
using PKL::DualQuaternion;
using PKL::Homogeneous;
using PKL::MatrixRP;

template<class TF_TYPE>
static TF_TYPE pkl_forward(const PKL::Chain<TF_TYPE>& chain, const PKL::Angles& theta) {
    return chain.forward(theta);
}

static KDL::Frame kdl_forward(KDL::ChainFkSolverPos& solver, const KDL::JntArray& jntArray) {
    KDL::Frame frame;
    solver.JntToCart(jntArray, frame, num_joints);
    KDL::Jacobian jac(7);
    return frame;
}

void BM_PKL_FORWARD_DQ(benchmark::State& state) {
    const auto chain = BENCHMARK_CONSTANTS::make_puma_pkl_chain<DualQuaternion>();
    PKL::Angles thetas = Eigen::Matrix<PKL::Scalar, num_joints, 1>::Zero();

    for (auto _ : state) {
        DualQuaternion forward = pkl_forward(chain, thetas);
        benchmark::DoNotOptimize(forward);
    }
}

void BM_PKL_FORWARD_HG(benchmark::State& state) {
    const auto chain = BENCHMARK_CONSTANTS::make_puma_pkl_chain<Homogeneous>();
    PKL::Angles thetas = Eigen::Matrix<PKL::Scalar, num_joints, 1>::Zero();

    for (auto _ : state) {
        Homogeneous forward = pkl_forward(chain, thetas);
        benchmark::DoNotOptimize(forward);
    }
}

void BM_PKL_FORWARD_RP(benchmark::State& state) {
    const auto chain = BENCHMARK_CONSTANTS::make_puma_pkl_chain<MatrixRP>();
    PKL::Angles thetas = Eigen::Matrix<PKL::Scalar, num_joints, 1>::Zero();

    for (auto _ : state) {
        MatrixRP forward = pkl_forward(chain, thetas);
        benchmark::DoNotOptimize(forward);
    }
}

void BM_KDL_FORWARD(benchmark::State& state) {
    auto chain = BENCHMARK_CONSTANTS::make_puma_kdl_chain();
    auto solver = KDL::ChainFkSolverPos_recursive(chain);
    KDL::JntArray jntArray(num_joints);
    KDL::Frame frame;

    for (auto _ : state) {
        frame = kdl_forward(solver, jntArray);
        benchmark::DoNotOptimize(frame);
    }
}

