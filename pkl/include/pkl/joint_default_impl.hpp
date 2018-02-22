#ifndef PKL_JOINT_DEFAULT_IMPL_HPP_
#define PKL_JOINT_DEFAULT_IMPL_HPP_

#include <Eigen/Geometry>
#include "pkl/typedef.h"
#include "pkl/matrix_rp.hpp"
#include "pkl/homogeneous.hpp"
#include "pkl/dual_quaternion.hpp"

namespace PKL {
template<class TF_TYPE>
struct DefaultImpl {
    template<JointType JOINT_TYPE>
    struct JointImpl : JointTypeTrait<JOINT_TYPE> {
        static TF_TYPE pose(Scalar theta, const Vec& axis, const TF_TYPE& offset);
        static Vec initialize_twist(const Vec& vec) {
            return Vec(vec);
        }
        static TwistVec adjoint(const TF_TYPE&, const Vec&);
    };
};

}  // namespace PKL
#endif  // PKL_JOINT_DEFAULT_IMPL_HPP_
