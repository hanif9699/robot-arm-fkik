#!/bin/sh
# Build against Orocos KDL + pybind11 (needs liborocos-kdl-dev, pybind11, python3-dev).
g++ -O2 -shared -fPIC -std=c++11 $(python3 -m pybind11 --includes) \
    -I/usr/include/eigen3 -I/usr/local/include RobotArmFKIK.cpp \
    -L/usr/local/lib -lorocos-kdl -o RobotArmFKIK$(python3-config --extension-suffix)
