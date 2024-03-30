#include "tools/kestrel-sim/src/SITLSimulator.h"
#include <iostream>
#include <cassert>

using namespace Kestrel::Sim;

void TestEndToEndAutonomousMission() {
    SITLSimulator sim;
    sim.Initialize();

    double dt = 0.002; // 500 Hz
    double maxDuration = 40.0;
    size_t steps = static_cast<size_t>(maxDuration / dt);

    double maxAltitude = 0.0;
    bool bReachedTakeoffAlt = false;

    for (size_t i = 0; i < steps; ++i) {
        sim.Step(dt);
        const auto& state = sim.GetState();

        double alt = -state.position.z;
        if (alt > maxAltitude) maxAltitude = alt;

        if (alt >= 14.0) {
            bReachedTakeoffAlt = true;
        }

        // Verify attitude stability: roll and pitch angles must not exceed 60 degrees (1.05 rad)
        double roll, pitch, yaw;
        state.attitude.ToEuler(roll, pitch, yaw);
        assert(std::abs(roll) < 1.05);
        assert(std::abs(pitch) < 1.05);

        // Verify battery remaining is strictly positive
        assert(sim.GetHistory().back().batteryPercent > 50.0);
    }

    assert(bReachedTakeoffAlt);
    assert(maxAltitude >= 14.5 && maxAltitude <= 22.0);
    assert(sim.GetHistory().size() == steps);

    std::cout << "  [PASS] 500 Hz End-to-End Autonomous Inspection Mission ("
              << steps << " ticks, Peak Alt: " << maxAltitude << "m)\n";
}

int main() {
    std::cout << "=== KESTREL TEST SUITE: END-TO-END SITL INTEGRATION MISSION ===\n";
    TestEndToEndAutonomousMission();
    std::cout << "ALL SITL INTEGRATION TESTS PASSED.\n\n";
    return 0;
}
