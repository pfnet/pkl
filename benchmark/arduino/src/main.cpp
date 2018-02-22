#include <Eigen/Core>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

#include <pkl/chain.hpp>
#include <pkl/dual_quaternion.hpp>
#include <pkl/homogeneous.hpp>
#include <pkl/matrix_rp.hpp>

#include "Arduino.h"
#include "BenchTimer.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

typedef PKL::MatrixRP TF_TYPE;

extern KDL::Chain make_puma_kdl_chain();
extern PKL::Chain<TF_TYPE> make_puma_pkl_chain();


static const int num_joints = 7;
static const PKL::Angles thetas = Eigen::Matrix<PKL::Scalar, num_joints, 1>::Random();
static const PKL::TwistVec desired_twist = PKL::TwistVec::Random();

static const int tries = 10;
static const int rep = 100;

float measureKDLForward() {
    KDL::Frame frame;
    KDL::Chain chain(make_puma_kdl_chain());
    KDL::JntArray jntArray(num_joints);
    KDL::ChainFkSolverPos_recursive solver(chain);

    Eigen::BenchTimer t;
    BENCH(t, tries, rep, solver.JntToCart(jntArray, frame, num_joints));
    return t.best(Eigen::CPU_TIMER) / rep;
}

float measureKDLJacobian() {
    KDL::Frame frame;
    KDL::Chain chain(make_puma_kdl_chain());
    KDL::JntArray jntArray(num_joints);
    KDL::Jacobian jac(num_joints);
    KDL::ChainJntToJacSolver solver(chain);

    Eigen::BenchTimer t;
    BENCH(t, tries, rep, solver.JntToJac(jntArray, jac, num_joints));
    return t.best(Eigen::CPU_TIMER) / rep;
}

float measureKDLInverse() {
    KDL::JntArray jntArray(num_joints);
    KDL::JntArray ret(num_joints);
    KDL::Frame frame;
    KDL::Chain chain(make_puma_kdl_chain());
    KDL::ChainIkSolverVel_pinv solver(chain);

    // for fair comparison, copy desired
    for (int i = 0; i < num_joints; ++i) {
        jntArray(i) = thetas(i);
    }
    KDL::Twist kdl_desired_twist(
        KDL::Vector(desired_twist(0), desired_twist(1), desired_twist(2)),
        KDL::Vector(desired_twist(3), desired_twist(4), desired_twist(5)));

    Eigen::BenchTimer t;
    BENCH(t, tries, rep, solver.CartToJnt(jntArray, kdl_desired_twist, ret));
    return t.best(Eigen::CPU_TIMER) / rep;
}

float measurePKLForward() {
    PKL::Chain<TF_TYPE> chain(make_puma_pkl_chain());

    Eigen::BenchTimer t;
    BENCH(t, tries, rep, chain.forward(thetas));
    return t.best(Eigen::CPU_TIMER) / rep;
}

float measurePKLJacobian() {
    PKL::Chain<TF_TYPE> chain(make_puma_pkl_chain());

    Eigen::BenchTimer t;
    BENCH(t, tries, rep, chain.jacobian(thetas));
    return t.best(Eigen::CPU_TIMER) / rep;
}

float measurePKLInverse() {
    PKL::Chain<TF_TYPE> chain(make_puma_pkl_chain());

    Eigen::BenchTimer t;
    BENCH(t, tries, rep, chain.inverse_vel(thetas, desired_twist));
    return t.best(Eigen::CPU_TIMER) / rep;
}

void setup()
{
    // initialize LED digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
}

void loop()
{
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);

  // wait for a second
  delay(1000);

  {
      Serial.print("'KDL', 'FORWARD', ");
      Serial.print(measureKDLForward());
      Serial.print(", 'Cortex-M3'\n'KDL', 'JACOBIAN', ");
      Serial.print(measureKDLJacobian());
      Serial.print(", 'Cortex-M3'\n'KDL', 'INVERSE', ");
      Serial.print(measureKDLInverse());
      Serial.print(", 'Cortex-M3'\n'PKL(RP)', 'FORWARD', ");
      Serial.print(measurePKLForward());
      Serial.print(", 'Cortex-M3'\n'PKL(RP)', 'JACOBIAN', ");
      Serial.print(measurePKLJacobian());
      Serial.print(", 'Cortex-M3'\n'PKL(RP)', 'INVERSE', ");
      Serial.print(measurePKLInverse());
      Serial.print(", 'Cortex-M3'\n");
  }

  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);

   // wait for a second
  delay(1000);
}
