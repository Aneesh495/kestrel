#pragma once

#include "Math/KestrelMathTypes.h"
#include <random>

namespace Kestrel {

struct WindParameters {
    Vector3 baseWindVelocity{2.0, 1.0, 0.0}; // Steady wind vector (m/s)
    double referenceAltitude{10.0};           // Anemometer reference height (m)
    double shearExponent{0.14};               // Terrain wind shear power coefficient (alpha)
    double turbulenceIntensity{0.15};         // Turbulence intensity (sigma / V)
    double gustAmplitude{8.0};                // Discrete gust peak velocity (m/s)
    double gustLength{40.0};                  // Discrete gust half-wavelength (m)
    uint32_t randomSeed{42};
};

/**
 * Atmospheric Wind and Turbulence Generation Engine.
 * Implements MIL-F-8785C / FAA discrete 1-cosine gusts, atmospheric power-law boundary layer shear,
 * and Dryden continuous spectral turbulence filter states.
 */
class KestrelWindModel {
public:
    KestrelWindModel();
    explicit KestrelWindModel(const WindParameters& inParams);

    /**
     * Resets turbulence filters and random seed.
     */
    void Reset();

    /**
     * Evaluates total wind velocity vector at vehicle position and time.
     *
     * @param vehiclePosition Inertial position (m)
     * @param airspeed Vehicle forward speed relative to air (m/s)
     * @param dt Time delta (s)
     * @return Total 3D wind velocity vector in inertial frame (m/s)
     */
    Vector3 SampleWind(const Vector3& vehiclePosition, double airspeed, double dt);

    /**
     * Triggers a discrete 1-cosine gust disturbance along the specified direction.
     *
     * @param direction Unit vector along gust direction
     * @param peakSpeed Maximum gust velocity (m/s)
     * @param lengthMeters Distance over which gust acts (m)
     */
    void TriggerDiscreteGust(const Vector3& direction, double peakSpeed, double lengthMeters);

private:
    WindParameters params;
    std::mt19937 rng;
    std::normal_distribution<double> gaussianDist{0.0, 1.0};

    // Dryden continuous filter internal state variables
    double stateU{0.0};
    double stateV1{0.0};
    double stateV2{0.0};
    double stateW1{0.0};
    double stateW2{0.0};

    // Discrete gust tracking
    bool bGustActive{false};
    Vector3 gustDirection{1.0, 0.0, 0.0};
    double gustPeak{0.0};
    double gustLengthMeters{0.0};
    double gustTraversedDistance{0.0};

    Vector3 ComputeTurbulence(double altitude, double airspeed, double dt);
};

} // namespace Kestrel
