#include <kdl/chain.hpp>
#include <kdl/joint.hpp>
#include <pkl/chain.hpp>
#include <pkl/dual_quaternion.hpp>
#include <pkl/homogeneous.hpp>

static const float d3 = 0.31;
static const float d5 = 0.31;
static const float d7 = 0.16;

typedef PKL::MatrixRP TF_TYPE;

KDL::Chain make_puma_kdl_chain() {
    KDL::Chain chain;
    const auto ey = KDL::Vector(0.0, 1.0, 0.0);
    const auto ez = KDL::Vector(0.0, 0.0, 1.0);
    chain.addSegment(KDL::Segment(KDL::Joint(
        KDL::Vector(0.0, 0.0, 0.0), ez, KDL::Joint::JointType::RotAxis)));
    chain.addSegment(KDL::Segment(KDL::Joint(
        KDL::Vector(0.0, 0.0, 0.0), ey, KDL::Joint::JointType::RotAxis)));
    chain.addSegment(KDL::Segment(KDL::Joint(
        KDL::Vector(0.0, 0.0, d3), ez, KDL::Joint::JointType::RotAxis)));
    chain.addSegment(KDL::Segment(KDL::Joint(
        KDL::Vector(0.0, 0.0, d3), ey, KDL::Joint::JointType::RotAxis)));
    chain.addSegment(KDL::Segment(KDL::Joint(
        KDL::Vector(0.0, 0.0, d3 + d5), ez, KDL::Joint::JointType::RotAxis)));
    chain.addSegment(KDL::Segment(KDL::Joint(
        KDL::Vector(0.0, 0.0, d3 + d5), ey, KDL::Joint::JointType::RotAxis)));
    chain.addSegment(KDL::Segment(KDL::Joint(
        KDL::Vector(0.0, 0.0, d3 + d5 + d7), ez, KDL::Joint::JointType::RotAxis)));
    return chain;
}

static TF_TYPE make_offset(PKL::Scalar x, PKL::Scalar y, PKL::Scalar z) {
    return TF_TYPE::PureTranslation({x, y, z});
}

PKL::Chain<TF_TYPE> make_puma_pkl_chain() {
    PKL::Chain<TF_TYPE> chain;
    chain.add_joint(PKL::JointTypeRev, PKL::Vec(0.0, 0.0, 1.0), make_offset(0.0, 0.0, 0.0));
    chain.add_joint(PKL::JointTypeRev, PKL::Vec(0.0, 1.0, 0.0), make_offset(0.0, 0.0, 0.0));
    chain.add_joint(PKL::JointTypeRev, PKL::Vec(0.0, 0.0, 1.0), make_offset(0.0, 0.0, d3));
    chain.add_joint(PKL::JointTypeRev, PKL::Vec(0.0, 1.0, 0.0), make_offset(0.0, 0.0, 0.0));
    chain.add_joint(PKL::JointTypeRev, PKL::Vec(0.0, 0.0, 1.0), make_offset(0.0, 0.0, d5));
    chain.add_joint(PKL::JointTypeRev, PKL::Vec(0.0, 1.0, 0.0), make_offset(0.0, 0.0, 0.0));
    chain.add_joint(PKL::JointTypeRev, PKL::Vec(0.0, 0.0, 1.0), make_offset(0.0, 0.0, d7));
    return chain;
}
