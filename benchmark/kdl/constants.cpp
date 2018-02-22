#include <pkl/typedef.h>
#include "constants.hpp"

namespace BENCHMARK_CONSTANTS {
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
}  // namespace BENCHMARK_CONSTANTS
