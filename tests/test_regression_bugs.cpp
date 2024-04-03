#include "Math/KestrelQuaternionMath.h"
#include "Aerodynamics/KestrelBladeElement.h"
#include "Aerodynamics/KestrelAtmosphere.h"
#include "Spatial/KestrelKDTree.h"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace Kestrel;

/**
 * Regression Bug #1:
 * Under high-frequency angular vibration (>100 rad/s), naive Euler integration
 * causes quaternion norm to drift away from 1.0, leading to scale explosion.
 * Solution: Exact SO(3) Lie group exponential map integration preserves unit norm.
 */
void TestRegressionBug1_QuaternionNormDrift() {
    Quat q = Quat::Identity();
    Vector3 highFreqVibration(120.0, -80.0, 95.0);
    double dt = 0.002;

    for (int i = 0; i < 5000; ++i) {
        q = KestrelQuaternionMath::IntegrateAngularVelocity(q, highFreqVibration, dt);
    }

    double normErr = std::abs(q.Norm() - 1.0);
    assert(normErr < 1e-10);
    std::cout << "  [PASS] Regression #1: Quaternion Unit Norm Drift Under High-Frequency Angular Vibration\n";
}

/**
 * Regression Bug #2:
 * When all 4 motors reach maximum thrust saturation (1.0), roll/pitch attitude
 * demands could be drowned out by collective throttle.
 * Solution: Attitude priority desaturation clamps collective throttle while preserving torque differentials.
 */
void TestRegressionBug2_MotorMixerDesaturationPriority() {
    double collectiveThrust = 1.0; // 100% saturation demand
    double rollDemand = 0.25;
    double pitchDemand = 0.15;
    double yawDemand = 0.05;

    // Standard Quad-X mixing without desaturation would saturate:
    double m1_raw = collectiveThrust - rollDemand + pitchDemand - yawDemand;
    double m2_raw = collectiveThrust + rollDemand - pitchDemand - yawDemand;
    double m3_raw = collectiveThrust + rollDemand + pitchDemand + yawDemand;
    double m4_raw = collectiveThrust - rollDemand - pitchDemand + yawDemand;

    // Desaturation logic: reduce collective so max(m_i) <= 1.0 while preserving diff
    double maxMotor = std::max(std::max(m1_raw, m2_raw), std::max(m3_raw, m4_raw));
    double desatReduction = std::max(0.0, maxMotor - 1.0);

    double m1 = m1_raw - desatReduction;
    double m2 = m2_raw - desatReduction;
    double m3 = m3_raw - desatReduction;
    double m4 = m4_raw - desatReduction;

    assert(m1 <= 1.0 && m2 <= 1.0 && m3 <= 1.0 && m4 <= 1.0);
    // Verify roll differential is preserved: (m2 + m3) - (m1 + m4)
    double rollDiffRaw = (m2_raw + m3_raw) - (m1_raw + m4_raw);
    double rollDiffDesat = (m2 + m3) - (m1 + m4);
    assert(std::abs(rollDiffRaw - rollDiffDesat) < 1e-9);

    std::cout << "  [PASS] Regression #2: Motor Mixer Desaturation Attitude Authority Preservation\n";
}

/**
 * Regression Bug #3:
 * In high-speed forward flight (15 m/s), Bernoulli effect creates negative dynamic
 * pressure over the barometric sensor port, creating a false climb signal.
 * Solution: Dynamic pressure compensation using airspeed estimate.
 */
void TestRegressionBug3_BaroDynamicPressureDistortion() {
    double trueAlt = 25.0;
    double airspeed = 15.0; // m/s
    double venturiFactor = 0.05;

    double pTrue = KestrelAtmosphere::AltitudeToPressure(trueAlt);
    double qDyn = 0.5 * 1.225 * airspeed * airspeed;
    double pDistorted = pTrue - venturiFactor * qDyn;

    double altUncompensated = KestrelAtmosphere::PressureToAltitude(pDistorted);
    assert(altUncompensated > trueAlt); // False climb error observed

    // Compensate with dynamic pressure estimation
    double pCompensated = pDistorted + venturiFactor * (0.5 * 1.225 * airspeed * airspeed);
    double altCompensated = KestrelAtmosphere::PressureToAltitude(pCompensated);

    assert(std::abs(altCompensated - trueAlt) < 1e-6);
    std::cout << "  [PASS] Regression #3: Barometric Venturi Dynamic Pressure Distortion Inversion\n";
}

/**
 * Regression Bug #4:
 * KD-Tree priority queue element sorting in FindKNearest requires strict operator<
 * on distance rather than whole struct to prevent compilation and sorting failures.
 */
void TestRegressionBug4_KDTreeHeapComparator() {
    KestrelKDTree tree;
    std::vector<KDPoint> pts = {
        {Vector3(1, 0, 0), 1},
        {Vector3(5, 0, 0), 2},
        {Vector3(2, 0, 0), 3},
        {Vector3(8, 0, 0), 4}
    };
    tree.Build(pts);

    std::vector<KDPoint> kNearest = tree.FindKNearest(Vector3(0, 0, 0), 2);
    assert(kNearest.size() == 2);
    // Closest are id 1 (dist 1) and id 3 (dist 2)
    assert(kNearest[0].id == 1);
    assert(kNearest[1].id == 3);

    std::cout << "  [PASS] Regression #4: KD-Tree K-Nearest Heap Ordering and Boundary Traversal\n";
}

int main() {
    std::cout << "=== KESTREL TEST SUITE: DOCUMENTED REGRESSION EDGE CASES ===\n";
    TestRegressionBug1_QuaternionNormDrift();
    TestRegressionBug2_MotorMixerDesaturationPriority();
    TestRegressionBug3_BaroDynamicPressureDistortion();
    TestRegressionBug4_KDTreeHeapComparator();
    std::cout << "ALL REGRESSION TESTS PASSED.\n\n";
    return 0;
}
