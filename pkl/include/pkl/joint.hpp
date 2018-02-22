#ifndef PKL_JOINT_HPP_
#define PKL_JOINT_HPP_

#include <cassert>
#include <Eigen/Geometry>
#include "pkl/typedef.h"
#include "pkl/matrix_rp.hpp"

namespace PKL {
enum JointType {JointTypeRev, JointTypePrism, JointTypeRigid};

template<JointType JOINT_TYPE>
struct JointTypeTrait {
    static bool is_rigid() {
        return false;
    }
};

template<>
struct JointTypeTrait<JointTypeRigid> {
    static bool is_rigid() {
        return true;
    }
};

template<class TF_TYPE>
struct JointTwistTrait {
    typedef Vec JointTwistType;
};

template<class TF_TYPE>
class Joint {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    template<class JOINT_IMPL>
    Joint(
        const Vec& axis,
        const TF_TYPE& offset,
        const JOINT_IMPL&):
        _axis(axis),
        _offset(offset),
        _twist(JOINT_IMPL::initialize_twist(axis)),
        _is_rigid(JOINT_IMPL::is_rigid()),
        _pose_impl(JOINT_IMPL::pose),
        _adjoint_impl(JOINT_IMPL::adjoint)
    {
    }

    template<class JOINT_IMPL>
    Joint(
        const TF_TYPE& offset,
        const JOINT_IMPL&):
        _offset(offset),
        _twist(JOINT_IMPL::initialize_twist(_axis)),
        _is_rigid(JOINT_IMPL::is_rigid)
    {
        _pose_impl = JOINT_IMPL::pose;
        // Only rigid joints are permitted for this constructor
        assert(_is_rigid);
    }

    TF_TYPE pose(Scalar theta) const {
        return (*_pose_impl)(theta, _axis, _offset);
    }

    TwistVec adjoint(const TF_TYPE& tf) const {
        return (*_adjoint_impl)(tf, _twist);
    }

    bool is_rigid() const {
        return _is_rigid;
    }

 protected:
    // don't allow users to directly instantiate Joints
    Joint() {}

    TF_TYPE _offset;
    typename JointTwistTrait<TF_TYPE>::JointTwistType _twist;
    Vec _axis;

    TF_TYPE (*_pose_impl)(Scalar, const Vec&, const TF_TYPE& offset);
    TwistVec (*_adjoint_impl)(const TF_TYPE&, const typename JointTwistTrait<TF_TYPE>::JointTwistType&);
    const bool _is_rigid;
};
}  // namespace PKL
#endif  // PKL_JOINT_HPP_
