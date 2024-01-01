#pragma once

#include "Math/KestrelMathTypes.h"
#include <random>

namespace Kestrel {

struct AllanVarianceParams {
    double angleRandomWalk{0.005};       // N: White noise spectral density (rad / s / sqrt(Hz))
    double rateRandomWalk{0.00005};      // K: Bias drift diffusion (rad / s^2 / sqrt(Hz))
    double biasInstability{0.0002};      // B: Pink noise flicker floor (rad / s)
    double scaleFactorError{0.001};      // Parts-per-million linear scale factor error
    double crossAxisCoupling{0.002};     // Cross-axis misalignment coupling
};

/**
 * Stochastic Sensor Error and Allan Variance Simulation Engine.
 * Generates calibrated Angle Random Walk (ARW), Rate Random Walk (RRW) bias drift,
 * scale factor errors, and motor mechanical vibration spectra for MEMS sensors.
 */
class KestrelNoiseModel {
public:
    explicit KestrelNoiseModel(const AllanVarianceParams& inParams = AllanVarianceParams(),
                               uint32_t seed = 1337);

    void Reset();

    /**
     * Applies Allan variance noise, bias drift, and vibrational harmonics to true measurement.
     *
     * @param trueRate True 3-axis angular velocity or linear acceleration
     * @param dt Time delta (s)
     * @param motorRpm Average motor speed generating vibrational noise (RPM)
     * @return Corrupted measurement vector
     */
    Vector3 ApplyIMUNoise(const Vector3& trueRate, double dt, double motorRpm = 0.0);

    /**
     * Returns the current accumulated bias vector.
     */
    Vector3 GetCurrentBias() const { return currentBias; }

    /**
     * Samples scalar Gaussian white noise with given standard deviation.
     */
    double SampleGaussian(double stdDev);

private:
    AllanVarianceParams params;
    std::mt19937 rng;
    std::normal_distribution<double> normalDist{0.0, 1.0};

    Vector3 currentBias{0.001, -0.002, 0.0015}; // Initial turn-on bias
    double vibrationPhase{0.0};
};

} // namespace Kestrel
