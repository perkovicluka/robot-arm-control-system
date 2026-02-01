#include <iostream>
#include <vector>
#include <cmath>

#include "kinematics.hpp"

/* 
    Euclidean distance between two 2D points.

    - We run inverseKinematics() to get joint-angle solutions for a target (x, y).
    - We plug each solution back into forwardKinematics() to recover the (x, y) position.
    - The distance between the recovered position and the original target is the FK/IK error.

    A small error indicates the IK math and FK math are consistent
    (up to floating-point rounding - need more ECE 204 Numerical Methods kowledge).
*/
double distance(const Vec2& a, const Vec2& b) {

    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);

}

/*
    The kinematics pipeline:

    1) Choose an arm geometry (L1, L2).
    2) Provide a small set of Cartesian targets to reach.
    3) For each target:
        - compute inverse kinematics (IK) solutions in joint space,
        - if reachable, verify each solution by running forward kinematics (FK),
        - print the recovered position and the numeric error.
*/
int main() {

    /*
        Simple 2-link planar arm model.

        L1 and L2 are the link lengths (in arbitrary units). Follows 3 cases (2 solutions, 1 solution, none).
    */
    Arm2D arm{1.0, 1.0};

    /*
        Cartesian target positions for the end-effector.

        Targets:
        - reachable interior points (produce elbow-up/down)
        - a boundary point at full extension (produce a single solution)
        - an unreachable point outside the workspace (produce no solution)
    */
    std::vector<Vec2> targets = {{1.0, 0.0}, {0.5, 0.5}, {0.0, 2.0}, {3.0, 0.0}, {1.0, 1.0}};

    for (const auto& target : targets) {

        // Print the target and compute IK solutions for it.

        std::cout << "Target (" << target.x << ", " << target.y << ")\n";
        IKResult res = inverseKinematics(arm, target);

        if (!res.reachable) {

            std::cout << "reachable: false\n\n";
            continue;

        }

        std::cout << "reachable: true\n\n";

        for (std::size_t i = 0; i < res.solutions.size(); ++i) {
            
            const auto& q = res.solutions[i];

            // Verify the IK result by running FK on the joint angles.
            // If FK(q) returns the original target (within tolerance), the solution is consistent.
            Vec2 p = forwardKinematics(arm, q);

            // Numeric error between the recovered FK position and the intended target.
            double err = distance(p, target);

            std::cout << "solution " << i + 1 << ":" << std::endl;;
            std::cout << "theta1 = " << q.theta1 << std::endl;
            std::cout << "theta2 = " << q.theta2 << std::endl;
            std::cout << "fk (" << p.x << ", " << p.y << "), error=" << err << "\n\n";

        }

        std::cout << "\n";

    }

    return 0;

}
