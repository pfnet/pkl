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

namespace {
TEST(ForwardTest, ForwardPUMADQ) {
    PKL::Chain<DualQuaternion> chain(make_puma_chain<DualQuaternion>());
    PKL::Angles thetas(7);
    thetas << 0.0, 0.0, 0.0, 0.5 * M_PI, 0.0, 0.0, 0.0;
    Eigen::Matrix<PKL::Scalar, 3, 4> expected_homogeneous;
    expected_homogeneous <<
        0, 0, 1, 0.47,
        0, 1, 0, 0.0,
        -1, 0, 0, 0.31;

    EXPECT_TRUE(is_close(
        chain.forward(thetas).to_homogeneous().affine(),
        expected_homogeneous));
}

TEST(ForwardTest, ForwardPUMARP) {
    PKL::Chain<MatrixRP> chain(make_puma_chain<MatrixRP>());
    PKL::Angles thetas(7);
    thetas << 0.0, 0.0, 0.0, 0.5 * M_PI, 0.0, 0.0, 0.0;
    Eigen::Matrix<PKL::Scalar, 3, 3> expected_rot;
    expected_rot <<
        0, 0, 1,
        0, 1, 0,
        -1, 0, 0;

    Eigen::Matrix<PKL::Scalar, 3, 1> expected_trans;
    expected_trans << 0.47, 0.0, 0.31;

    EXPECT_TRUE(is_close(chain.forward(thetas).get_rot(), expected_rot));
    EXPECT_TRUE(is_close(chain.forward(thetas).get_trans(), expected_trans));
}

TEST(ForwardTest, ForwardPrismDQ) {
    PKL::Chain<DualQuaternion> chain;
    chain.add_joint(PKL::JointTypePrism, Vec(2.0, 0.0, 0.0), vec2dq({0.0, 0.0, 1.0}));
    chain.add_joint(PKL::JointTypePrism, Vec(0.0, 3.0, 0.0), vec2dq({0.0, 10.0, 0.0}));
    Eigen::Matrix<PKL::Scalar, 2, 1> thetas;
    thetas << 10.0, 10.0;
    DualQuaternion expected_tf(DualQuaternion::PureTranslation({20.0, 40.0, 1.0}));
    EXPECT_TRUE(is_close(
        expected_tf,
        chain.forward(thetas)));
}

TEST(ForwardTest, ForwardPrismHG) {
    PKL::Chain<Homogeneous> chain;
    chain.add_joint(PKL::JointTypePrism, Vec(2.0, 0.0, 0.0), vec2hg({0.0, 0.0, 1.0}));
    chain.add_joint(PKL::JointTypePrism, Vec(0.0, 3.0, 0.0), vec2hg({0.0, 10.0, 0.0}));
    Eigen::Matrix<PKL::Scalar, 2, 1> thetas;
    thetas << 10.0, 10.0;
    Eigen::Matrix<PKL::Scalar, 4, 4> expected_tf;
    expected_tf <<
        1.0, 0.0, 0.0, 20.0,
        0.0, 1.0, 0.0, 40.0,
        0.0, 0.0, 1.0, 01.0,
        0.0, 0.0, 0.0, 01.0;
    EXPECT_TRUE(is_close(
        expected_tf,
        chain.forward(thetas).mat()));
}

/*
TEST(ChainTest, AssertionTestForward) {
    // check if assertion fails.
    // Is there a way to check this automatically?
    static const Vec axis(1.0, 2.0, 3.0);
    static const Vec origin(0.0, 2.0, 4.0);

    PKL::Chain<DualQuaternion> chain;
    chain.add_joint(PKL::JointTypePrism, Vec(0.0, 0.0, 0.0), DualQuaternion());
    chain.add_joint(PKL::JointTypePrism, Vec(0.0, 0.0, 0.0), DualQuaternion());

    Eigen::Matrix<PKL::Scalar, 1, 1> thetas = Eigen::Matrix<PKL::Scalar, 1, 1>::Zero();

    // error. we have two joints but receive just one angle
    chain.forward(thetas);
}
*/
}  // namespace
