#include "Math/KestrelQuaternionMath.h"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace Kestrel;

void TestQuaternionIdentity() {
    Quat q = Quat::Identity();
    assert(std::abs(q.w - 1.0) < 1e-9);
    assert(std::abs(q.x) < 1e-9);
    assert(std::abs(q.y) < 1e-9);
    assert(std::abs(q.z) < 1e-9);
    assert(KestrelQuaternionMath::IsNormalized(q));
    std::cout << "  [PASS] Quaternion Identity and Normalization\n";
}

void TestEulerConversionsAndGimbalLockAvoidance() {
    // Test arbitrary orientation
    double roll = 0.25;
    double pitch = -0.45;
    double yaw = 1.35;

    Quat q = Quat::FromEuler(roll, pitch, yaw);
    assert(KestrelQuaternionMath::IsNormalized(q));

    double outRoll, outPitch, outYaw;
    q.ToEuler(outRoll, outPitch, outYaw);

    assert(std::abs(roll - outRoll) < 1e-6);
    assert(std::abs(pitch - outPitch) < 1e-6);
    assert(std::abs(yaw - outYaw) < 1e-6);

    // Test extreme pitch (gimbal pole: +90 deg and -90 deg)
    Quat qPole = Quat::FromEuler(0.0, PI * 0.5, 0.0);
    qPole.ToEuler(outRoll, outPitch, outYaw);
    assert(std::abs(outPitch - PI * 0.5) < 1e-6);

    Quat qPoleNeg = Quat::FromEuler(0.0, -PI * 0.5, 0.0);
    qPoleNeg.ToEuler(outRoll, outPitch, outYaw);
    assert(std::abs(outPitch + PI * 0.5) < 1e-6);

    std::cout << "  [PASS] Euler Conversion and Gimbal Pole Singularity Avoidance\n";
}

void TestSO3ExpLogRoundtrip() {
    Vector3 rotationVec(0.15, -0.25, 0.45);
    Quat q = KestrelQuaternionMath::Exp(rotationVec);
    assert(KestrelQuaternionMath::IsNormalized(q));

    Vector3 recoveredVec = KestrelQuaternionMath::Log(q);
    assert(std::abs(rotationVec.x - recoveredVec.x) < 1e-6);
    assert(std::abs(rotationVec.y - recoveredVec.y) < 1e-6);
    assert(std::abs(rotationVec.z - recoveredVec.z) < 1e-6);

    // Near-zero rotation vector (Taylor expansion branch)
    Vector3 tinyVec(1e-9, -2e-9, 1.5e-9);
    Quat qTiny = KestrelQuaternionMath::Exp(tinyVec);
    assert(KestrelQuaternionMath::IsNormalized(qTiny));
    Vector3 recoveredTiny = KestrelQuaternionMath::Log(qTiny);
    assert((tinyVec - recoveredTiny).Norm() < 1e-8);

    std::cout << "  [PASS] SO(3) Lie Group Exp/Log Roundtrip and Small-Angle Limit\n";
}

void TestSlerpShortestPath() {
    Quat q0 = Quat::Identity();
    Quat q1 = Quat::FromEuler(0.0, 0.0, 0.5); // 0.5 rad yaw

    Quat qMid = KestrelQuaternionMath::Slerp(q0, q1, 0.5);
    double r, p, y;
    qMid.ToEuler(r, p, y);
    assert(std::abs(y - 0.25) < 1e-4);

    // Test antipodal representation (q and -q represent same orientation in SO(3))
    Quat q1Antipodal(-q1.w, -q1.x, -q1.y, -q1.z);
    Quat qMidAntipodal = KestrelQuaternionMath::Slerp(q0, q1Antipodal, 0.5);
    qMidAntipodal.ToEuler(r, p, y);
    assert(std::abs(y - 0.25) < 1e-4);

    std::cout << "  [PASS] SLERP Shortest-Path Geodesic and Antipodal Disambiguation\n";
}

void TestVectorRotation() {
    // Rotate [1, 0, 0] by 90 degrees yaw -> should point to [0, 1, 0]
    Quat qYaw90 = Quat::FromEuler(0.0, 0.0, PI * 0.5);
    Vector3 v(1.0, 0.0, 0.0);
    Vector3 vRot = qYaw90.Rotate(v);

    assert(std::abs(vRot.x) < 1e-6);
    assert(std::abs(vRot.y - 1.0) < 1e-6);
    assert(std::abs(vRot.z) < 1e-6);

    // Inverse rotation
    Vector3 vInv = qYaw90.RotateInverse(vRot);
    assert(std::abs(vInv.x - 1.0) < 1e-6);
    assert(std::abs(vInv.y) < 1e-6);
    assert(std::abs(vInv.z) < 1e-6);

    std::cout << "  [PASS] 3D Vector Quaternion Rotation and Inverse Rotation\n";
}

int main() {
    std::cout << "=== KESTREL TEST SUITE: QUATERNION MATH & SO(3) KINEMATICS ===\n";
    TestQuaternionIdentity();
    TestEulerConversionsAndGimbalLockAvoidance();
    TestSO3ExpLogRoundtrip();
    TestSlerpShortestPath();
    TestVectorRotation();
    std::cout << "ALL QUATERNION KINEMATICS TESTS PASSED.\n\n";
    return 0;
}
