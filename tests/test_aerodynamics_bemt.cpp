#include "Aerodynamics/KestrelBladeElement.h"
#include "Aerodynamics/KestrelAtmosphere.h"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace Kestrel;

void TestBEMTThrustScaling() {
    KestrelBladeElement bemt;

    RotorState r1;
    r1.omegaRadPerS = 400.0; // rad/s
    r1.groundClearanceMeters = 5.0; // Out of ground effect
    RotorAerodynamicOutput out1 = bemt.ComputeRotorForces(r1, 1.225);

    RotorState r2;
    r2.omegaRadPerS = 800.0; // Double RPM
    r2.groundClearanceMeters = 5.0;
    RotorAerodynamicOutput out2 = bemt.ComputeRotorForces(r2, 1.225);

    // Thrust scales quadratically: T2 / T1 should be approx 4.0
    double thrustRatio = out2.thrustNewtons / out1.thrustNewtons;
    assert(std::abs(thrustRatio - 4.0) < 0.25);
    assert(out1.thrustNewtons > 0.5); // Produces real positive lift

    // Torque should also scale quadratically
    double torqueRatio = out2.torqueNewtonMeters / out1.torqueNewtonMeters;
    assert(std::abs(torqueRatio - 4.0) < 0.25);

    std::cout << "  [PASS] BEMT Quadratic Thrust and Torque Scaling (Ratio: " << thrustRatio << " ~ 4.0)\n";
}

void TestGroundEffectMultiplier() {
    KestrelBladeElement bemt;

    // In-ground-effect at 0.08m height (R = 0.127m, ratio ~ 0.397)
    double igeClose = bemt.ComputeGroundEffect(0.08);
    assert(igeClose > 1.10 && igeClose < 1.45);

    // Out-of-ground-effect at 5.0m height
    double igeFar = bemt.ComputeGroundEffect(5.0);
    assert(std::abs(igeFar - 1.0) < 0.005);

    // Monotonic decay: lower ground clearance produces higher ground effect
    double igeMid = bemt.ComputeGroundEffect(0.35);
    assert(igeClose > igeMid && igeMid > igeFar);

    std::cout << "  [PASS] Cheeseman-Bennett In-Ground-Effect Lift Cushion (IGE: " << igeClose << " -> " << igeFar << ")\n";
}

void TestAtmosphericDensityLapse() {
    AtmosphericState sl = KestrelAtmosphere::GetStateAtAltitude(0.0);
    assert(std::abs(sl.densityKgPerM3 - 1.225) < 0.001);
    assert(std::abs(sl.pressurePascals - 101325.0) < 1.0);
    assert(std::abs(sl.temperatureKelvin - 288.15) < 0.01);

    AtmosphericState alt3k = KestrelAtmosphere::GetStateAtAltitude(3000.0);
    assert(alt3k.densityKgPerM3 < sl.densityKgPerM3);
    assert(alt3k.densityKgPerM3 > 0.85 && alt3k.densityKgPerM3 < 0.95);

    // Hypsometric inversion
    double calculatedAlt = KestrelAtmosphere::PressureToAltitude(alt3k.pressurePascals);
    assert(std::abs(calculatedAlt - 3000.0) < 0.5);

    std::cout << "  [PASS] US Standard Atmosphere 1976 and Hypsometric Inversion\n";
}

int main() {
    std::cout << "=== KESTREL TEST SUITE: AERODYNAMICS & BLADE ELEMENT MOMENTUM ===\n";
    TestBEMTThrustScaling();
    TestGroundEffectMultiplier();
    TestAtmosphericDensityLapse();
    std::cout << "ALL AERODYNAMICS AND PROPULSION TESTS PASSED.\n\n";
    return 0;
}
