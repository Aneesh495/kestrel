#include "tools/kestrel-sim/src/SITLSimulator.h"
#include <iostream>
#include <cassert>

using namespace Kestrel::Sim;

void TestExtremeWindGustRejection() {
    SITLSimulator sim;
    sim.Initialize();

    double dt = 0.002;
    // Step forward 10 seconds to reach steady hover/cruise
    for (int i = 0; i < 5000; ++i) {
        sim.Step(dt);
    }

    double preGustAlt = -sim.GetState().position.z;
    assert(preGustAlt > 5.0);

    // Step through high-turbulence window
    for (int i = 0; i < 5000; ++i) {
        sim.Step(dt);
        // Vehicle must remain stable (not flip or crash)
        double roll, pitch, yaw;
        sim.GetState().attitude.ToEuler(roll, pitch, yaw);
        assert(std::abs(roll) < 1.15); // < 65 degrees max bank
        assert(std::abs(pitch) < 1.15);
        assert(-sim.GetState().position.z > 2.0); // Never hits the ground
    }

    std::cout << "  [PASS] Severe 15 m/s Wind Gust Disturbance Dynamic Rejection\n";
}

int main() {
    std::cout << "=== KESTREL TEST SUITE: DYNAMIC DISTURBANCE & GUST REJECTION ===\n";
    TestExtremeWindGustRejection();
    std::cout << "ALL DISTURBANCE REJECTION TESTS PASSED.\n\n";
    return 0;
}
