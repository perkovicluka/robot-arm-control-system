#pragma once

#include <vector>
#include <cmath>

// Simple 2D vector for positions.
struct Vec2 {
    double x;
    double y;
};

// Two joint angles for the planar arm.
struct JointAngles {
    double theta1;
    double theta2;
};

// Output of IK: whether the point is reachable and the angle solutions.
struct IKResult {
    bool reachable;
    std::vector<JointAngles> solutions;
};

// Link lengths of the planar arm.
struct Arm2D {
    double L1;
    double L2;
};

// Given angles, return the (x, y) position of the end.
Vec2 forwardKinematics(const Arm2D& arm, const JointAngles& q);

// Given a target (x, y), return all joint solutions that work.
IKResult inverseKinematics(const Arm2D& arm, const Vec2& target);
