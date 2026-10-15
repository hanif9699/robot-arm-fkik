// Reconstructed source of the RobotArmFKIK Python extension
// ("FK and IK Orocos based python binding 2018 by rvatcha").
// Rebuilt by reverse engineering the compiled module; behaviour (including
// quirks such as degree conversion of every joint and the debug printing in
// computeInverseKinematics) mirrors the original binary.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;

class RobotArmMechanism {
public:
    RobotArmMechanism() : dof(0) {}

    // Add a segment with an identity frame, e.g. "RotZ", "TransX" or "None".
    void addSegment(const std::string &axis) {
        KDL::Segment seg(makeJoint(axis), KDL::Frame::Identity(), zeroInertia());
        append(axis, seg);
    }

    // Add a segment described by Craig (1989) DH parameters; the angles
    // alpha and theta are given in degrees.
    void addSegment(const std::string &axis, double a, double alpha, double d, double theta) {
        const double k = KDL::PI / 180.0;
        KDL::Segment seg(makeJoint(axis), KDL::Frame::DH_Craig1989(a, alpha * k, d, theta * k),
                         zeroInertia());
        append(axis, seg);
    }

    void finaliseChain() {
        fk.reset(new KDL::ChainFkSolverPos_recursive(chain));
        ikvel.reset(new KDL::ChainIkSolverVel_pinv(chain, 1e-5, 150));
        ik.reset(new KDL::ChainIkSolverPos_NR(chain, *fk, *ikvel, 100, 1e-6));
    }

    // Joint values in degrees -> 4x4 homogeneous transform.
    std::vector<std::vector<double>> computeForwardKinematics(const std::vector<double> &angles) {
        KDL::JntArray q(chain.getNrOfJoints());
        for (int i = 0; i < dof; ++i) q(i) = angles[i] * KDL::PI / 180.0;

        KDL::Frame f;
        fk->JntToCart(q, f);

        std::vector<std::vector<double>> m(4, std::vector<double>(4, 0.0));
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) m[r][c] = f.M(r, c);
            m[r][3] = f.p(r);
        }
        m[3][3] = 1.0;
        return m;
    }

    // target: 4x4 transform, guess: initial joint values (degrees),
    // out: receives the solution in degrees wrapped to [-180, 180].
    bool computeInverseKinematics(const std::vector<std::vector<double>> &target,
                                  const std::vector<double> &guess, py::array_t<double> out) {
        std::cout << "Entered here....";
        KDL::Frame goal(KDL::Rotation(target[0][0], target[0][1], target[0][2],
                                      target[1][0], target[1][1], target[1][2],
                                      target[2][0], target[2][1], target[2][2]),
                        KDL::Vector(target[0][3], target[1][3], target[2][3]));

        KDL::JntArray qInit(chain.getNrOfJoints());
        for (int i = 0; i < dof; ++i) qInit(i) = guess[i] * KDL::PI / 180.0;
        if (qInit(1) > -60.0 * KDL::PI / 180.0) qInit(1) = -KDL::PI;

        KDL::JntArray qOut(chain.getNrOfJoints());
        int ret = ik->CartToJnt(qInit, goal, qOut);
        bool ok = (ret != -1);

        std::cout << "Entered here2....";
        std::vector<double> deg(dof);
        for (int i = 0; i < dof; ++i) deg[i] = qOut(i) * 180.0 / KDL::PI;
        for (int i = 0; i < dof; ++i) deg[i] = wrap(deg[i]);

        auto buf = out.mutable_unchecked<1>();
        for (int i = 0; i < dof; ++i) {
            std::cout << deg[i] << " ";
            buf(i) = deg[i];
        }
        return ok;
    }

    std::string repr() const { return "Robot arm with DOF = " + std::to_string(dof); }

private:
    KDL::Chain chain;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ikvel;
    std::unique_ptr<KDL::ChainIkSolverPos_NR> ik;
    int dof;

    static KDL::RigidBodyInertia zeroInertia() {
        return KDL::RigidBodyInertia(0.0, KDL::Vector::Zero(),
                                     KDL::RotationalInertia(0, 0, 0, 0, 0, 0));
    }

    static KDL::Joint makeJoint(const std::string &axis) {
        using JT = KDL::Joint::JointType;
        if (axis.find("Rot", 0, 3) != std::string::npos) {
            if (axis.find("X", 0, 1) != std::string::npos) return KDL::Joint(JT::RotX);
            if (axis.find("Y", 0, 1) != std::string::npos) return KDL::Joint(JT::RotY);
            if (axis.find("Z", 0, 1) != std::string::npos) return KDL::Joint(JT::RotZ);
            throw std::domain_error(
                "invalid rotation axis specified.. Pass string as RotX or RotY or RotZ");
        }
        if (axis.find("Trans", 0, 5) != std::string::npos) {
            if (axis.find("X", 0, 1) != std::string::npos) return KDL::Joint(JT::TransX);
            if (axis.find("Y", 0, 1) != std::string::npos) return KDL::Joint(JT::TransY);
            if (axis.find("Z", 0, 1) != std::string::npos) return KDL::Joint(JT::TransZ);
            throw std::domain_error(
                "Invalid translation axis specified.. Pass string as TransX or TransY or TransZ");
        }
        if (axis.find("None", 0, 4) != std::string::npos) return KDL::Joint(JT::None);
        throw std::domain_error("Invalid axis specified.. Pass string as RotX or RotY or RotZ or "
                                "TransX or TransY or TransZ or None");
    }

    void append(const std::string &axis, const KDL::Segment &seg) {
        chain.addSegment(seg);
        if (seg.getJoint().getType() != KDL::Joint::None) ++dof;
        (void)axis;
    }

    // Reduce an angle in degrees to [-180, 180] (360 is applied in float,
    // as in the original).
    static double wrap(double a) {
        if (a < 0.0)
            a += (double)((float)(int)std::floor(-a / 360.0) * 360.0f);
        else
            a -= (double)((float)(int)std::floor(a / 360.0) * 360.0f);
        if (a >= -180.0) {
            if (a > 180.0) {
                a -= 360.0;
                if (a < -180.0) a += 360.0;
            }
        } else {
            a += 360.0;
        }
        return a;
    }
};

PYBIND11_MODULE(RobotArmFKIK, m) {
    m.doc() = "FK and IK Orocos based python binding 2018 by rvatcha";
    py::module k = m.def_submodule("Kinematics", "FK and IK based on DH parameters");
    py::class_<RobotArmMechanism>(k, "RobotArmMechanism", "Robot arm kinematics FK and IK")
        .def(py::init<>())
        .def("addSegment", (void (RobotArmMechanism::*)(const std::string &)) &RobotArmMechanism::addSegment)
        .def("addSegment", (void (RobotArmMechanism::*)(const std::string &, double, double, double, double)) &RobotArmMechanism::addSegment)
        .def("finaliseChain", &RobotArmMechanism::finaliseChain)
        .def("computeForwardKinematics", &RobotArmMechanism::computeForwardKinematics)
        .def("computeInverseKinematics", &RobotArmMechanism::computeInverseKinematics)
        .def("__repr__", &RobotArmMechanism::repr);
}
