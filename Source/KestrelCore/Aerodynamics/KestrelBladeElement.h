#pragma once

#include "Math/KestrelMathTypes.h"

namespace Kestrel {

struct RotorState {
    double omegaRadPerS{0.0};              // Rotor angular velocity
    Vector3 localAirflowVelocity{0.0,0.0,0.0}; // Local relative wind velocity at rotor hub
    double groundClearanceMeters{5.0};     // Height above terrain for ground effect (IGE)
};

struct RotorAerodynamicOutput {
    double thrustNewtons{0.0};             // Net axial thrust force
    double torqueNewtonMeters{0.0};        // Aerodynamic reaction torque (yaw axis)
    Vector3 hubForceH{0.0, 0.0, 0.0};      // In-plane drag and H-force from blade flapping
    double inducedInflowVelocity{0.0};     // Induced downwash velocity vi (m/s)
    double groundEffectMultiplier{1.0};    // IGE thrust magnification factor
};

/**
 * Blade Element Momentum Theory (BEMT) Aerodynamic Solver.
 * Solves non-linear rotor inflow, blade advance ratios, H-force drag,
 * and Cheeseman-Bennett ground effect for multirotor propulsion.
 */
class KestrelBladeElement {
public:
    struct PropellerGeometry {
        double radius{0.127};              // Propeller radius (5 inches = 0.127 m)
        double chord{0.018};               // Mean blade chord (m)
        double pitch{0.1016};              // Blade pitch (4 inches = 0.1016 m)
        int bladeCount{2};                 // Number of propeller blades
        double liftSlope{5.73};            // Section lift curve slope Cla (1/rad)
        double zeroLiftDragCoeff{0.012};   // Section profile drag coefficient Cd0
        int spinDirection{1};              // +1 = Clockwise (CW), -1 = Counter-Clockwise (CCW)
    };

    explicit KestrelBladeElement(const PropellerGeometry& inGeom = PropellerGeometry());

    /**
     * Evaluates aerodynamic forces and moments produced by a single rotor.
     *
     * @param rotor Current operational rotor state
     * @param airDensity Ambient atmospheric air density rho (kg/m^3)
     * @return Aerodynamic thrust, torque, and in-plane forces
     */
    RotorAerodynamicOutput ComputeRotorForces(const RotorState& rotor, double airDensity) const;

    /**
     * Computes Cheeseman-Bennett In-Ground-Effect (IGE) thrust magnification ratio.
     *
     * @param heightAboveGround Distance from rotor disk to terrain (m)
     * @return Multiplier >= 1.0
     */
    double ComputeGroundEffect(double heightAboveGround) const;

private:
    PropellerGeometry geom;
    double diskArea;

    double SolveInducedInflow(double thrustTarget, double airDensity, double axialVelocity) const;
};

} // namespace Kestrel
