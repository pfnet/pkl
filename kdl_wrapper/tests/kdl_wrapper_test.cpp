#include <gtest/gtest.h>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <pkl/kdl_wrapper/chainiksolvervel_pinv.hpp>
#include <pkl/kdl_wrapper/converter.hpp>

bool is_close(const Eigen::Matrix<PKL::Scalar, 6, -1>& x, const Eigen::Matrix<PKL::Scalar, 6, -1>& y) {
    static const PKL::Scalar eps = 1E-6;
    return (x - y).norm() < eps;
}

KDL::Chain chain_from_baxtrer_model() {
    KDL::Chain chain;
    KDL::Tree tree;
    std::string chain_start = "torso";
    std::string chain_stop = "torso";
    std::string chain_end =  "right_hand";
    urdf::Model robot_model;
    assert(robot_model.initFile("kdl_wrapper/tests/models/baxter.urdf"));
    assert(kdl_parser::treeFromUrdfModel(robot_model, tree));
    assert(tree.getChain(chain_start, chain_end, chain));
    /* trac_ik test implementation that I don't understand, but isn't required for trac_ik benchmarks
    KDL::Chain ret;
    for (size_t i = 0; i < chain.getNrOfSegments(); ++i) {
        if (!tree.getChain(chain.segments[i].getName(), chain_end, ret)) {
            std::cout << "error4" << std::endl;
        }
        if (chain.getNrOfJoints() != ret.getNrOfJoints()) {
            break;
        }
        if (ret.getNrOfJoints() == 0) {
            break;
        }
    }
    std::cout << "initialized chain" << std::endl;
    */
    return chain;
}

KDL::Chain make_puma_kdl_chain() {
    static const PKL::Scalar d3 = 0.31;
    static const PKL::Scalar d5 = 0.31;
    static const PKL::Scalar d7 = 0.16;
    KDL::Chain chain;
    const auto ey = KDL::Vector(0.0, 1.0, 0.0);
    const auto ez = KDL::Vector(0.0, 0.0, 1.0);
    chain.addSegment(KDL::Segment(KDL::Joint(
        KDL::Vector(0.0, 0.0, 0.0), ez, KDL::Joint::JointType::RotAxis),
        KDL::Frame({0.0, 0.0, 0.0})));
    chain.addSegment(KDL::Segment(KDL::Joint(
        KDL::Vector(0.0, 0.0, 0.0), ey, KDL::Joint::JointType::RotAxis),
        KDL::Frame({0.0, 0.0, 0.0})));
    chain.addSegment(KDL::Segment(KDL::Joint(
        KDL::Vector(0.0, 0.0, 0.0), ez, KDL::Joint::JointType::RotAxis),
        KDL::Frame({0.0, 0.0, d3})));
    chain.addSegment(KDL::Segment(KDL::Joint(
        KDL::Vector(0.0, 0.0, 0.0), ey, KDL::Joint::JointType::RotAxis),
        KDL::Frame({0.0, 0.0, 0.0})));
    chain.addSegment(KDL::Segment(KDL::Joint(
        KDL::Vector(0.0, 0.0, 0.0), ez, KDL::Joint::JointType::RotAxis),
        KDL::Frame({0.0, 0.0, d5})));
    chain.addSegment(KDL::Segment(KDL::Joint(
        KDL::Vector(0.0, 0.0, 0.0), ey, KDL::Joint::JointType::RotAxis),
        KDL::Frame({0.0, 0.0, 0.0})));
    chain.addSegment(KDL::Segment(KDL::Joint(
        KDL::Vector(0.0, 0.0, 0.0), ez, KDL::Joint::JointType::RotAxis),
        KDL::Frame({0.0, 0.0, d7})));
    return chain;
}

KDL::Chain make_two_link_chain() {
    KDL::Chain chain;

    const auto ey = KDL::Vector(0.0, 1.0, 0.0);
    chain.addSegment(KDL::Segment(
        KDL::Joint(KDL::Vector(0.1, 0.2, 0.3), ey, KDL::Joint::JointType::RotAxis),
        KDL::Frame(KDL::Vector(1.0, 2.0, 3.0))));
    chain.addSegment(KDL::Segment(
        KDL::Joint(KDL::Vector(0.1, 0.2, 0.3), ey, KDL::Joint::JointType::RotAxis),
        KDL::Frame(KDL::Vector(1.0, 2.0, 3.0))));
    chain.addSegment(KDL::Segment(
        KDL::Joint(KDL::Vector(0.01, 0.02, 0.03), ey, KDL::Joint::JointType::RotAxis),
        KDL::Frame(KDL::Vector(10.0, 20.0, 30.0))));
    return chain;
}

KDL::JntArray make_jntarray(int num_joints) {
    KDL::JntArray ret(num_joints);
    for (int i = 0; i < num_joints; ++i)
        ret(i) = 0.1 * i;
    return ret;
}

bool test_jacobian(const KDL::Chain& chain) {
    auto pkl_chain = PKL::KDL_WRAPPER::kdl_chain_to_eigen(chain);
    const int num_joints = pkl_chain.num_non_rigid_joints();

    KDL::JntArray qin = make_jntarray(num_joints);
    KDL::JntArray qout(num_joints);
    KDL::Twist desired(KDL::Vector(0.0, 0.1, 0.2), KDL::Vector(0.3, 0.4, 0.5));


    // KDL
    KDL::ChainJntToJacSolver jacsolver(chain);
    KDL::Jacobian jac(num_joints);
    jacsolver.JntToJac(qin, jac);

    return is_close(
        jac.data.cast<PKL::Scalar>(),
        pkl_chain.jacobian(PKL::KDL_WRAPPER::kdl_jnt_to_angles(qin)).cast<PKL::Scalar>());
}

namespace {
TEST(KDLWrapperTest, JacobianPumaTest) {
    EXPECT_TRUE(test_jacobian(make_puma_kdl_chain()));
}
}  // namespace
