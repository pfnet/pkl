#include <gtest/gtest.h>
#include <Eigen/Core>
#include "utils.hpp"
#include "pkl/chain.hpp"
#include "pkl/dual_quaternion.hpp"
#include "pkl/homogeneous.hpp"
#include "pkl/matrix_rp.hpp"

using PKL::DualQuaternion;
using PKL::Homogeneous;
using PKL::MatrixRP;
using PKL::Vec;

template<class TF_TYPE>
bool test_jacobian() {
    PKL::Chain<TF_TYPE> chain(make_puma_chain<TF_TYPE>());
    Eigen::Matrix<PKL::Scalar, num_joints, 1> thetas;
    thetas << 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
    Eigen::Matrix<PKL::Scalar, 6, num_joints> expected_jacobian;
    expected_jacobian <<
        -0.0691298, +0.6967875, -0.0687845, +0.3792242, -0.0409139, +0.0764387, +0.0000000,
        +0.2615394, +0.0000000, +0.1906702, +0.0813386, +0.0635750, +0.0736321, +0.0000000,
        +0.0000000, -0.2615394, +0.0069015, -0.2396600, +0.0129771, -0.1197307, +0.0000000,
        +0.0000000, +0.0000000, +0.0998334, -0.1976768, +0.3835570, -0.5333718, +0.6980525,
        +0.0000000, +1.0000000, +0.0000000, +0.9800666, +0.0587108, +0.8287910, +0.3183093,
        +1.0000000, +0.0000000, +0.9950042, +0.0198338, +0.9216491, +0.1691745, +0.6414062;

    const bool ret = is_close(
        expected_jacobian,
        chain.jacobian(thetas));
    return ret;
}

namespace {
TEST(JacobianTest, JacobianPUMADQ) {
    EXPECT_TRUE(test_jacobian<DualQuaternion>());
}

TEST(JacobianTest, JacobianPUMAHG) {
    EXPECT_TRUE(test_jacobian<Homogeneous>());
}

TEST(JacobianTest, JacobianPUMARP) {
    EXPECT_TRUE(test_jacobian<MatrixRP>());
}

TEST(JacobianTest, JacobianPrism) {
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
}  // namespace
