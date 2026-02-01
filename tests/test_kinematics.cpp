#include <cassert>
#include <cmath>
#include <vector>

#include "kinematics.hpp"

/*
    Euclidean distance between two 2D points.

    Used to verify FK/IK round-trip accuracy within a tolerance.
*/
double dist(const Vec2& a, const Vec2& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

/*
    IK reachability test.

    Target outside |L1 - L2| <= r <= L1 + L2 should be unreachable.
    Expected: reachable == false, no solutions.
*/
void test_unreachable() {
    Arm2D arm{1.0, 1.0};
    Vec2 target{3.0, 0.0};
    IKResult res = inverseKinematics(arm, target);
    assert(!res.reachable);
    assert(res.solutions.empty());
}

/*
    IK -> FK consistency test.

    For each reachable target:
    - solve IK
    - run FK on each solution
    - FK result should match target within tolerance
*/
void test_fk_matches_targets() {
    Arm2D arm{1.0, 1.0};
    const double tol = 1e-6;
    std::vector<Vec2> targets = {{1.0, 0.0}, {0.5, 0.5}, {0.0, 2.0}};

    for (const auto& target : targets) {
        IKResult res = inverseKinematics(arm, target);
        assert(res.reachable);
        for (const auto& q : res.solutions) {
            Vec2 p = forwardKinematics(arm, q);
            assert(dist(p, target) < tol);
        }
    }
}

/*
    Singularity (full extension) test:

    When the target lies exactly at r = L1 + L2 (arm fully stretched), elbow-up and elbow-down
    collapse into the same physical pose (theta2 = 0), so only one unique solution should be
    returned.

    Expected:
    - reachable == true
    - exactly one solution
    - FK(solution) matches target within tolerance
*/
void test_full_extension_singularity() {
    Arm2D arm{1.0, 1.0};
    const double tol = 1e-6;
    Vec2 target{2.0, 0.0};
    IKResult res = inverseKinematics(arm, target);
    assert(res.reachable);
    assert(res.solutions.size() == 1);
    Vec2 p = forwardKinematics(arm, res.solutions.front());
    assert(dist(p, target) < tol);
}

/*
    Tests

    - unreachable targets
    - FK/IK consistency
    - straight-arm singularity
*/
int main() {
    test_unreachable();
    test_fk_matches_targets();
    test_full_extension_singularity();
    return 0;
}
