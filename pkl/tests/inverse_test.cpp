#include <gtest/gtest.h>
#include <Eigen/Core>
#include "utils.hpp"
#include "pkl/chain.hpp"
#include "pkl/dual_quaternion.hpp"
#include "pkl/homogeneous.hpp"
#include "pkl/matrix_rp.hpp"

using PKL::DualQuaternion;
using PKL::Homogeneous;
using PKL::Vec;

template<class TF_TYPE>
bool test_inverse() {
    PKL::Chain<TF_TYPE> chain(make_puma_chain<TF_TYPE>());
    Eigen::Matrix<PKL::Scalar, num_joints, 1> thetas = Eigen::Matrix<PKL::Scalar, num_joints, 1>::Random();
    PKL::TwistVec desired = PKL::TwistVec::Random();
    Eigen::Matrix<PKL::Scalar, 6, num_joints> jacobian = chain.jacobian(thetas);
    bool ret = is_close(
        jacobian * chain.inverse_vel(thetas, desired),
        desired);
    return ret;
}

namespace {
TEST(InverseTest, InversePUMADQ) {
    EXPECT_TRUE(test_inverse<DualQuaternion>());
}

TEST(InverseTest, InversePUMAHG) {
    EXPECT_TRUE(test_inverse<Homogeneous>());
}

TEST(InverseTest, JacobianPrism) {
    static const PKL::Scalar scale = 10.0;
    static const Vec axis(1.0, 2.0, 3.0);
    static const Vec origin(0.0, 2.0, 4.0);
    PKL::Chain<DualQuaternion> chain;
    chain.add_joint(PKL::JointTypePrism, axis, vec2dq(origin));
    Eigen::Matrix<PKL::Scalar, 1, 1> thetas;
    thetas << scale;
    Eigen::Matrix<PKL::Scalar, 6, 1> expected_jacobian;
    expected_jacobian <<
        axis(0),
        axis(1),
        axis(2),
        0.0, 0.0, 0.0;
    EXPECT_TRUE(is_close(
        expected_jacobian,
        chain.jacobian(thetas)));
}

TEST(InverseTest, InversePrism) {
    static const Vec axis(1.0, 2.0, 3.0);
    static const Vec origin(0.0, 2.0, 4.0);
    PKL::Chain<DualQuaternion> chain;
    chain.add_joint(PKL::JointTypePrism, axis, vec2dq(origin));
    Eigen::Matrix<PKL::Scalar, 1, 1> thetas = Eigen::Matrix<PKL::Scalar, 1, 1>::Zero();
    PKL::TwistVec desired;
    desired << 1.0, 2.0, 3.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<PKL::Scalar, 1, 1> expected_angles;
    expected_angles << 1.0;
    EXPECT_TRUE(is_close(
        chain.inverse_vel(thetas, desired),
        expected_angles));
}
}  // namespace
