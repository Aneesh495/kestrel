#include "KestrelBladeElement.h"
#include <algorithm>

namespace Kestrel {

KestrelBladeElement::KestrelBladeElement()
    : KestrelBladeElement(PropellerGeometry()) {}

KestrelBladeElement::KestrelBladeElement(const PropellerGeometry& inGeom)
    : geom(inGeom)
    , diskArea(PI * inGeom.radius * inGeom.radius) {}

double KestrelBladeElement::ComputeGroundEffect(double heightAboveGround) const {
    if (heightAboveGround <= 0.05) {
        return 1.45; // Maximum physical cushion clamp
    }
    double ratio = geom.radius / (4.0 * heightAboveGround);
    if (ratio >= 0.9) return 1.45;

    // Cheeseman-Bennett formulation: T_IGE / T_OGE = 1 / (1 - (R / (4*z))^2)
    double factor = 1.0 / (1.0 - ratio * ratio);
    return std::clamp(factor, 1.0, 1.45);
}

double KestrelBladeElement::SolveInducedInflow(double thrustTarget, double airDensity, double axialVelocity) const {
    if (thrustTarget <= 0.0 || airDensity <= 0.0) return 0.0;

    // Hover induced velocity: v_h = sqrt(T / (2 * rho * A))
    double vHover = std::sqrt(thrustTarget / (2.0 * airDensity * diskArea));
    double vi = vHover;

    // Fixed-point Newton iteration for general inflow: vi = T / (2 * rho * A * sqrt(Vz^2 + vi^2))
    for (int iter = 0; iter < 10; ++iter) {
        double vTotal = std::sqrt(axialVelocity * axialVelocity + vi * vi);
        if (vTotal < 1e-4) break;
        double f = vi - thrustTarget / (2.0 * airDensity * diskArea * vTotal);
        double fPrime = 1.0 + (thrustTarget * vi) / (2.0 * airDensity * diskArea * vTotal * vTotal * vTotal);
        double step = f / fPrime;
        vi -= step;
        if (std::abs(step) < 1e-5) break;
    }
    return std::max(0.0, vi);
}

RotorAerodynamicOutput KestrelBladeElement::ComputeRotorForces(const RotorState& rotor, double airDensity) const {
    RotorAerodynamicOutput out;
    if (std::abs(rotor.omegaRadPerS) < 1.0) {
        return out;
    }

    double tipSpeed = std::abs(rotor.omegaRadPerS) * geom.radius;
    double solidity = (geom.bladeCount * geom.chord) / (PI * geom.radius);

    // Dynamic pressure at 75% blade radius
    double v75 = 0.75 * tipSpeed;
    double q75 = 0.5 * airDensity * v75 * v75;

    // Effective angle of attack: theta_pitch - induced_flow_angle
    double geometricPitchAngle = std::atan2(geom.pitch, 2.0 * PI * 0.75 * geom.radius);
    double axialVelocity = -rotor.localAirflowVelocity.z; // Downwash direction
    double inflowAngle = std::atan2(axialVelocity, v75);
    double alphaEff = geometricPitchAngle - inflowAngle;

    // Rotor thrust coefficient Ct
    double cl = geom.liftSlope * alphaEff;
    double ct = 0.5 * solidity * cl * (1.0 / 3.0); // Blade integration over radius
    ct = std::max(0.0, ct);

    double baseThrust = ct * airDensity * diskArea * tipSpeed * tipSpeed;

    // Ground effect scaling
    out.groundEffectMultiplier = ComputeGroundEffect(rotor.groundClearanceMeters);
    out.thrustNewtons = baseThrust * out.groundEffectMultiplier;

    // Solve for induced downwash velocity vi
    out.inducedInflowVelocity = SolveInducedInflow(out.thrustNewtons, airDensity, axialVelocity);

    // Aerodynamic torque: profile drag + induced drag
    double cd = geom.zeroLiftDragCoeff + (cl * cl) / (PI * 3.0);
    double cq = 0.5 * solidity * (cd * 0.25 + ct * (out.inducedInflowVelocity / tipSpeed));
    out.torqueNewtonMeters = cq * airDensity * diskArea * tipSpeed * tipSpeed * geom.radius * geom.spinDirection;

    // In-plane H-force (horizontal drag caused by advancing blade dynamic pressure asymmetry)
    double advanceRatioX = rotor.localAirflowVelocity.x / tipSpeed;
    double advanceRatioY = rotor.localAirflowVelocity.y / tipSpeed;
    double hForceCoeff = 0.5 * solidity * geom.zeroLiftDragCoeff * 0.5;

    out.hubForceH.x = -hForceCoeff * airDensity * diskArea * tipSpeed * tipSpeed * advanceRatioX;
    out.hubForceH.y = -hForceCoeff * airDensity * diskArea * tipSpeed * tipSpeed * advanceRatioY;
    out.hubForceH.z = 0.0;

    return out;
}

} // namespace Kestrel
