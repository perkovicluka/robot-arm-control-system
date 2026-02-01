#include "kinematics.hpp"
#include <algorithm>

/*
    Forward kinematics (FK)

    Input:  joint angles (theta1, theta2)
    Output: end position (x, y)

    Link 1 points at theta1.
    Link 2 points at (theta1 + theta2).
    Tip = link1 vector + link2 vector.
*/
Vec2 forwardKinematics(const Arm2D& arm, const JointAngles& q) {

    Vec2 p;
    p.x = arm.L1 * std::cos(q.theta1) + arm.L2 * std::cos(q.theta1 + q.theta2);
    p.y = arm.L1 * std::sin(q.theta1) + arm.L2 * std::sin(q.theta1 + q.theta2);
    return p;

}

/*
    Inverse kinematics (IK)

    Input:  target (x, y)
    Output: 0, 1, or 2 joint solutions (theta1, theta2)

    Steps:
    1) r^2 = x^2 + y^2
    2) Reach check: |L1 - L2| <= r <= L1 + L2
    3) cos(theta2) from law of cos from [-1, 1]
    4) sin(theta2) = +/- sqrt(1 - cos^2)
    5) theta1 = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2))

    - Two options: elbow-up (+sin) and elbow-down (-sin).
    - Straight arm (sin == 0) is unique solution.
*/
IKResult inverseKinematics(const Arm2D& arm, const Vec2& target) {

    IKResult result;

    // r = distance from base to target
    double r2 = target.x * target.x + target.y * target.y;
    double r = std::sqrt(r2);
    double L1 = arm.L1;
    double L2 = arm.L2;

    // Reachable workspace is in "ring of access"
    double minReach = std::fabs(L1 - L2);
    double maxReach = L1 + L2;
    if (r > maxReach || r < minReach) {

        result.reachable = false;
        return result;

    }

    // cos(theta2) from law of cosines (rounded to [-1, 1] bc of floating-point inaccuracy)
    double cosTheta2 = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    if (cosTheta2 > 1.0) {
        
        cosTheta2 = 1.0;

    }
    if (cosTheta2 < -1.0) {
        
        cosTheta2 = -1.0;
    
    }

    // sin(theta2) magnitude sign gives elbow-up or elbow-down
    double sinTheta2 = std::sqrt(std::max(0.0, 1.0 - cosTheta2 * cosTheta2));

    // Straight-arm case: sin(theta2) == 0 -> only one unique solution
    if (sinTheta2 == 0.0) {

        double theta2 = std::atan2(0.0, cosTheta2);

        // Solve theta1 using atan2 form to avoid edge cases
        double k1 = L1 + L2 * std::cos(theta2);
        double k2 = L2 * std::sin(theta2);
        double theta1 = std::atan2(target.y, target.x) - std::atan2(k2, k1);
        result.solutions.push_back({theta1, theta2});

    } else {

        // Elbow-up (+sin)
        double theta2a = std::atan2(sinTheta2, cosTheta2);
        double k1a = L1 + L2 * std::cos(theta2a);
        double k2a = L2 * std::sin(theta2a);
        double theta1a = std::atan2(target.y, target.x) - std::atan2(k2a, k1a);
        result.solutions.push_back({theta1a, theta2a});

        // Elbow-down (-sin)
        double theta2b = std::atan2(-sinTheta2, cosTheta2);
        double k1b = L1 + L2 * std::cos(theta2b);
        double k2b = L2 * std::sin(theta2b);
        double theta1b = std::atan2(target.y, target.x) - std::atan2(k2b, k1b);
        result.solutions.push_back({theta1b, theta2b});

    }

    // At least one solution was produced
    result.reachable = true;
    return result;
    
}
