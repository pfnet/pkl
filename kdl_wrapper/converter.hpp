#ifndef KDL_WRAPPER_CONVERTER_HPP_
#define KDL_WRAPPER_CONVERTER_HPP_

#include <Eigen/Core>
#include <kdl/frames.hpp>
#include <pkl/joint.hpp>
#include <pkl/typedef.h>

namespace PKL {
namespace KDL_WRAPPER{
inline Vec kdl_vec_to_eigen(const KDL::Vector& vec) {
    Vec ret;
    ret << vec(0), vec(1), vec(2);
    return ret;
}

inline Homogeneous kdl_frame_to_homogeneous(const KDL::Frame& frame) {
    Eigen::Matrix<Scalar, 4, 4> ret;
    ret <<
        frame(0, 0), frame(0, 1), frame(0, 2), frame(0, 3),
        frame(1, 0), frame(1, 1), frame(1, 2), frame(1, 3),
        frame(2, 0), frame(2, 1), frame(2, 2), frame(2, 3),
        frame(3, 0), frame(3, 1), frame(3, 2), frame(3, 3);
    return Homogeneous(ret);
}

inline DualQuaternion kdl_frame_to_dq(const KDL::Frame& frame) {
    double x, y, z, w;
    frame.M.GetQuaternion(x, y, z, w);
    Eigen::Quaternion<Scalar> rot;
    rot.coeffs() << x, y, z, w;
    Eigen::Quaternion<Scalar> trans(0, frame.p.x(), frame.p.y(), frame.p.z());
    trans.coeffs() = (trans * rot).coeffs() * 0.5;
    return DualQuaternion(rot, trans);
}

inline Angles kdl_jnt_to_angles(const KDL::JntArray& jntArray) {
    Eigen::Matrix<Scalar, -1, 1> ret(jntArray.rows(), 1);
    for (int i = 0; i < jntArray.rows(); ++i) {
        ret(i) = jntArray(i);
    }
    return ret;
}

inline KDL::JntArray angles_to_kdl_jnt(const Angles& angles) {
    KDL::JntArray ret(angles.rows());
    for (int i = 0; i < angles.rows(); ++i) {
        ret(i) = angles(i);
    }
    return ret;
}

inline TwistVec kdl_twist_to_vec(const KDL::Twist& twist) {
    TwistVec ret;
    ret <<
        twist.vel(0), twist.vel(1), twist.vel(2),
        twist.rot(0), twist.rot(1), twist.rot(2);
    return ret;
}

inline Chain<Homogeneous> kdl_chain_to_eigen(const KDL::Chain& chain) {
    Chain<Homogeneous> ret(
        kdl_frame_to_homogeneous(chain.getSegment(0).getJoint().pose(0)));

    for (size_t i = 0; i < chain.segments.size(); i++) {
        Vec axis = kdl_vec_to_eigen(
            chain.segments[i].getJoint().JointAxis());
        PKL::Homogeneous global_tf = kdl_frame_to_homogeneous(
            chain.getSegment(i).getJoint().pose(0)).inverse() * kdl_frame_to_homogeneous(
            chain.getSegment(i).pose(0));
        if (i + 1 != chain.segments.size()) {
            global_tf = global_tf * kdl_frame_to_homogeneous(
                chain.getSegment(i+1).getJoint().pose(0));
        }

        if (chain.getSegment(i).getJoint().getType() == KDL::Joint::None) {
            ret.add_rigid_joint(global_tf);
        } else {
            std::string type = chain.segments[i].getJoint().getTypeName();
            if (type.find("Rot") != std::string::npos) {
                ret.add_joint(PKL::JointTypeRev, axis, global_tf);
            } else if (type.find("Trans") != std::string::npos) {
                ret.add_joint(PKL::JointTypePrism, axis, global_tf);
            }
        }
    }
    return ret;
}
};  // namespace KDL_WRAPPER
};  // namespace PKL

#endif  // KDL_WRAPPER_CONVERTER_HPP_
