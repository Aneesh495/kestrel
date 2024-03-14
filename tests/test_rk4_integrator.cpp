#include "Math/KestrelRK4Integrator.h"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace Kestrel;

void TestHarmonicOscillatorEnergyConservation() {
    // Undamped 1D oscillator: d^2(x)/dt^2 = -omega^2 * x -> omega = 2.0 rad/s
    double omega = 2.0;
    auto harmonicDynamics = [omega](const RigidBodyState& s, double t) {
        (void)t;
        StateDerivative d;
        d.linearVelocity = s.velocity;
        d.linearAcceleration = -s.position * (omega * omega);
        return d;
    };

    RigidBodyState state;
    state.position = Vector3(1.0, 0.0, 0.0); // Initial displacement = 1.0 m
    state.velocity = Vector3(0.0, 0.0, 0.0); // Initial velocity = 0

    double initialEnergy = 0.5 * (omega * omega) * state.position.NormSq() + 0.5 * state.velocity.NormSq();

    double dt = 0.005; // 200 Hz
    int steps = 10000;  // 50 simulated seconds
    for (int i = 0; i < steps; ++i) {
        state = KestrelRK4Integrator::Step(state, dt, harmonicDynamics, i * dt);
    }

    double finalEnergy = 0.5 * (omega * omega) * state.position.NormSq() + 0.5 * state.velocity.NormSq();
    double energyErrorFraction = std::abs(finalEnergy - initialEnergy) / initialEnergy;

    // In RK4 over 50 seconds, energy drift must be < 1e-5
    assert(energyErrorFraction < 1e-5);
    std::cout << "  [PASS] RK4 Symplectic Energy Conservation (error: " << energyErrorFraction << ")\n";
}

void TestRK4FourthOrderConvergenceRate() {
    // Stiff non-linear differential equation: dy/dt = -y^2, exact solution y(t) = y0 / (1 + y0 * t)
    double y0 = 1.0;
    auto stiffDynamics = [](const RigidBodyState& s, double t) {
        (void)t;
        StateDerivative d;
        d.linearVelocity = Vector3(-s.position.x * s.position.x, 0.0, 0.0);
        return d;
    };

    double tFinal = 2.0;
    double exactY = y0 / (1.0 + y0 * tFinal); // exact = 1 / 3 = 0.333333

    // Step size 1: dt = 0.1
    double dt1 = 0.1;
    RigidBodyState s1;
    s1.position = Vector3(y0, 0, 0);
    for (int i = 0; i < static_cast<int>(tFinal / dt1); ++i) {
        s1 = KestrelRK4Integrator::Step(s1, dt1, stiffDynamics, i * dt1);
    }
    double err1 = std::abs(s1.position.x - exactY);

    // Step size 2: dt = 0.05 (half)
    double dt2 = 0.05;
    RigidBodyState s2;
    s2.position = Vector3(y0, 0, 0);
    for (int i = 0; i < static_cast<int>(tFinal / dt2); ++i) {
        s2 = KestrelRK4Integrator::Step(s2, dt2, stiffDynamics, i * dt2);
    }
    double err2 = std::abs(s2.position.x - exactY);

    // For 4th order method, error ratio err1 / err2 should be approx 2^4 = 16
    double orderEstimate = std::log2(err1 / err2);
    assert(orderEstimate >= 3.8); // Confirms O(dt^4) convergence
    std::cout << "  [PASS] RK4 Fourth-Order Convergence Rate (Order: " << orderEstimate << " ~ 4.0)\n";
}

void TestRigidBodyCoupledGyroscopicMoment() {
    DronePhysicalParams params;
    RigidBodyState state;
    state.angularVelocity = Vector3(0.0, 10.0, 10.0); // High pitch and yaw rates

    // Net applied torque = 0
    StateDerivative deriv = KestrelRK4Integrator::ComputeRigidBodyDerivative(
        state, Vector3::Zero(), Vector3::Zero(), params);

    // Euler's equation J*dw/dt + w x (J*w) = 0 implies dw/dt = -J^-1 * (w x J*w) != 0
    assert(deriv.angularAcceleration.Norm() > 0.0);
    std::cout << "  [PASS] Rigid-Body Cross-Coupled Gyroscopic Precession Derivatives\n";
}

int main() {
    std::cout << "=== KESTREL TEST SUITE: 4TH-ORDER RUNGE-KUTTA DYNAMICS ===\n";
    TestHarmonicOscillatorEnergyConservation();
    TestRK4FourthOrderConvergenceRate();
    TestRigidBodyCoupledGyroscopicMoment();
    std::cout << "ALL RUNGE-KUTTA INTEGRATOR TESTS PASSED.\n\n";
    return 0;
}
