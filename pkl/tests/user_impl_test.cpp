#include <gtest/gtest.h>
#include <Eigen/Core>
#include "utils.hpp"
#include "pkl/chain.hpp"
#include "pkl/joint.hpp"

using PKL::Vec;

class UserTransform {
 public:
    void shift_translation(const UserTransform& other) {
    }
    PKL::TwistVec to_twist_vec() const {
        return PKL::TwistVec::Zero();
    }
    UserTransform inverse() const {
        return UserTransform();
    }
    static UserTransform Identity() {
        return UserTransform();
    }
    friend UserTransform operator*(const UserTransform&, const UserTransform&) {
        return UserTransform();
    }
};

template<PKL::JointType JOINT_TYPE>
struct UserImpl : PKL::JointTypeTrait<JOINT_TYPE> {
    static UserTransform pose(PKL::Scalar, const Eigen::Matrix<PKL::Scalar, 3, 1>& axis, const UserTransform& tf) {
        return UserTransform();
    }
    static Vec initialize_twist(const Vec&) {
        return Vec::Zero();
    }
    static PKL::TwistVec adjoint(const UserTransform&, const Vec&) {
        return PKL::TwistVec::Zero();
    }
};

namespace {
TEST(UserImplTest, UserImpl) {
    // make sure users can use their joint implementation
    static const Vec axis(1.0, 2.0, 3.0);
    static const Vec origin(0.0, 2.0, 4.0);
    PKL::Chain<UserTransform> chain;
    chain.add_joint<UserImpl>(PKL::JointTypePrism, Vec(0.0, 0.0, 0.0), UserTransform());
    Eigen::Matrix<PKL::Scalar, 1, 1> thetas = Eigen::Matrix<PKL::Scalar, 1, 1>::Zero();

    // static check (do they compile?)
    chain.forward(thetas);
    chain.inverse_vel(thetas, PKL::TwistVec::Zero());

    EXPECT_TRUE(is_close(
        chain.jacobian(thetas),
        Eigen::Matrix<PKL::Scalar, 6, 1>::Zero()));
}
}  // namespace
