# RobotArmFKIK (reconstructed source)

pybind11 wrapper around Orocos KDL: `RobotArmFKIK.Kinematics.RobotArmMechanism`.
Rebuilt from the compiled 2018 module and verified against it with
`compare_with_original.py` (300 random chains: FK, IK result/return value/stdout,
error messages, docstrings all identical).

Build: `./build.sh` (needs KDL 1.4 + headers, Eigen3, pybind11, python3-dev).
Compare: `python3 compare_with_original.py <dir containing the module>`.

Behaviour (kept as in the original):
- All joint values are degrees, including prismatic (Trans*) joints.
- `addSegment(axis)` -> identity frame; `addSegment(axis,a,alpha,d,theta)` -> DH Craig1989 (alpha/theta in degrees).
- `finaliseChain()`: FK recursive, IK vel pinv (eps 1e-5, 150 iters), IK pos NR (100 iters, eps 1e-6).
- IK: seed joint 1 is forced to -180 deg if it is > -60 deg; output wrapped to [-180,180];
  prints "Entered here...." / "Entered here2...." and the angles to stdout.
- Differences: the original segfaults when destroyed before `finaliseChain()`; this version doesn't.
