#include "KestrelWindModel.h"
#include <algorithm>

namespace Kestrel {

KestrelWindModel::KestrelWindModel(const WindParameters& inParams)
    : params(inParams)
    , rng(inParams.randomSeed)
{
    Reset();
}

void KestrelWindModel::Reset() {
    rng.seed(params.randomSeed);
    stateU = 0.0;
    stateV1 = 0.0;
    stateV2 = 0.0;
    stateW1 = 0.0;
    stateW2 = 0.0;
    bGustActive = false;
    gustTraversedDistance = 0.0;
}

Vector3 KestrelWindModel::SampleWind(const Vector3& vehiclePosition, double airspeed, double dt) {
    if (dt <= 0.0) return params.baseWindVelocity;

    // Atmospheric altitude (NED convention: altitude is -Z)
    double altitude = std::max(1.0, -vehiclePosition.z);

    // 1. Mean wind velocity boundary layer profile (Hellmann power law)
    double heightRatio = altitude / params.referenceAltitude;
    double shearMultiplier = std::pow(std::clamp(heightRatio, 0.1, 50.0), params.shearExponent);
    Vector3 meanWind = params.baseWindVelocity * shearMultiplier;

    // 2. Continuous Dryden turbulence stochastic velocity
    Vector3 turbulence = ComputeTurbulence(altitude, std::max(1.0, airspeed), dt);

    // 3. Discrete 1-cosine gust velocity
    Vector3 gustVelocity = Vector3::Zero();
    if (bGustActive) {
        gustTraversedDistance += std::max(1.0, airspeed) * dt;
        if (gustTraversedDistance >= gustLengthMeters) {
            bGustActive = false;
        } else {
            // 1-cosine shape: V_gust = 0.5 * V_peak * (1 - cos(pi * d / d_m))
            double phase = PI * gustTraversedDistance / gustLengthMeters;
            double gustMag = 0.5 * gustPeak * (1.0 - std::cos(phase));
            gustVelocity = gustDirection * gustMag;
        }
    }

    return meanWind + turbulence + gustVelocity;
}

Vector3 KestrelWindModel::ComputeTurbulence(double altitude, double airspeed, double dt) {
    // Low altitude Dryden scale lengths (MIL-F-8785C)
    double h = std::clamp(altitude, 1.0, 300.0);
    double Lw = h;
    double Lu = h / std::pow(0.177 + 0.000823 * h, 1.2);
    double Lv = Lu;

    // Turbulence root-mean-square intensities
    double sigmaW = 0.1 * params.baseWindVelocity.Norm();
    double sigmaU = sigmaW / std::pow(0.177 + 0.000823 * h, 0.4);
    double sigmaV = sigmaU;

    // First-order filter for longitudinal turbulence u: d(u)/dt = -(V/Lu)*u + sigma_u*sqrt(2*V/Lu)*w_white
    double whiteNoiseU = gaussianDist(rng);
    double poleU = airspeed / Lu;
    double gainU = sigmaU * std::sqrt(2.0 * airspeed / Lu);
    stateU += (-poleU * stateU + gainU * whiteNoiseU) * dt;

    // Second-order filter for lateral turbulence v
    double whiteNoiseV = gaussianDist(rng);
    double poleV = airspeed / Lv;
    double gainV = sigmaV * std::sqrt(airspeed / Lv);
    stateV1 += stateV2 * dt;
    stateV2 += (-poleV * poleV * stateV1 - 2.0 * poleV * stateV2 + gainV * whiteNoiseV) * dt;
    double turbV = stateV1 + std::sqrt(3.0) * (Lv / airspeed) * stateV2;

    // Second-order filter for vertical turbulence w
    double whiteNoiseW = gaussianDist(rng);
    double poleW = airspeed / Lw;
    double gainW = sigmaW * std::sqrt(airspeed / Lw);
    stateW1 += stateW2 * dt;
    stateW2 += (-poleW * poleW * stateW1 - 2.0 * poleW * stateW2 + gainW * whiteNoiseW) * dt;
    double turbW = stateW1 + std::sqrt(3.0) * (Lw / airspeed) * stateW2;

    return Vector3(stateU, turbV, turbW);
}

void KestrelWindModel::TriggerDiscreteGust(const Vector3& direction, double peakSpeed, double lengthMeters) {
    bGustActive = true;
    gustDirection = direction.Normalized();
    gustPeak = peakSpeed;
    gustLengthMeters = std::max(1.0, lengthMeters);
    gustTraversedDistance = 0.0;
}

} // namespace Kestrel
