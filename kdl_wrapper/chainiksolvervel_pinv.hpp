#ifndef KDL_WRAPPER_CHAINIKSOLVERVEL_PINV_HPP_
#define KDL_WRAPPER_CHAINIKSOLVERVEL_PINV_HPP_

#include <string>
#include <kdl/chainiksolver.hpp>
#include <pkl/chain.hpp>
#include "converter.hpp"

namespace PKL {
namespace KDL_WRAPPER {
class ChainIkSolverVel_pinv: public KDL::ChainIkSolverVel
{
 public:
    explicit ChainIkSolverVel_pinv(const KDL::Chain& chain):
        _pkl_chain(kdl_chain_to_eigen(chain))
    {
    }
    ~ChainIkSolverVel_pinv() {
    }

    int CartToJnt(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out) override {
        auto angles = kdl_jnt_to_angles(q_in);
        auto desired_twist = kdl_twist_to_vec(v_in);
        auto solution = _pkl_chain.inverse_vel(angles, desired_twist);
        for (size_t i = 0; i < qdot_out.rows(); ++i) {
            qdot_out(i) = solution(i);
        }
        return 0;
    }

    int CartToJnt(const KDL::JntArray& q_init, const KDL::FrameVel& v_in, KDL::JntArrayVel& q_out) override {
        // not supported
        assert(false);
        return 0;
    }
    // for debugging
    PKL::Jacobian jacobian(const PKL::Angles& angles) const {
        return _pkl_chain.jacobian(angles);
    }
 private:
    PKL::Chain<PKL::Homogeneous> _pkl_chain;
};
}  // namespace KDL_WRAPPER
}  // namespace PKL

#endif  // KDL_WRAPPER_CHAINIKSOLVERVEL_PINV_HPP_
