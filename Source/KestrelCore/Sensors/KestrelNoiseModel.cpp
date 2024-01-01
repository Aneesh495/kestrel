#include "KestrelNoiseModel.h"
#include <cmath>

namespace Kestrel {

KestrelNoiseModel::KestrelNoiseModel(const AllanVarianceParams& inParams, uint32_t seed)
    : params(inParams)
    , rng(seed)
{
    Reset();
}

void KestrelNoiseModel::Reset() {
    currentBias = Vector3(0.001, -0.002, 0.0015);
    vibrationPhase = 0.0;
}

double KestrelNoiseModel::SampleGaussian(double stdDev) {
    if (stdDev <= 0.0) return 0.0;
    return normalDist(rng) * stdDev;
}

Vector3 KestrelNoiseModel::ApplyIMUNoise(const Vector3& trueRate, double dt, double motorRpm) {
    if (dt <= 0.0) return trueRate;

    // 1. Scale factor and cross-axis misalignment matrix
    Matrix3x3 scaleMatrix(
        1.0 + params.scaleFactorError, params.crossAxisCoupling, -params.crossAxisCoupling,
        -params.crossAxisCoupling, 1.0 - params.scaleFactorError, params.crossAxisCoupling,
        params.crossAxisCoupling, -params.crossAxisCoupling, 1.0 + params.scaleFactorError
    );
    Vector3 scaledRate = scaleMatrix * trueRate;

    // 2. White noise (Angle Random Walk N): sigma = N / sqrt(dt)
    double whiteNoiseStd = params.angleRandomWalk / std::sqrt(dt);
    Vector3 whiteNoise(SampleGaussian(whiteNoiseStd),
                       SampleGaussian(whiteNoiseStd),
                       SampleGaussian(whiteNoiseStd));

    // 3. Bias random walk (Rate Random Walk K): delta_b = K * sqrt(dt) * w_k
    double biasWalkStd = params.rateRandomWalk * std::sqrt(dt);
    currentBias.x += SampleGaussian(biasWalkStd);
    currentBias.y += SampleGaussian(biasWalkStd);
    currentBias.z += SampleGaussian(biasWalkStd);

    // 4. Motor mechanical vibration (spectral harmonic at blade pass frequency)
    Vector3 vibrationNoise = Vector3::Zero();
    if (motorRpm > 100.0) {
        double bladePassFreq = (motorRpm / 60.0) * 4.0; // 4-blade passing frequency
        vibrationPhase += 2.0 * PI * bladePassFreq * dt;
        if (vibrationPhase > 2.0 * PI) vibrationPhase -= 2.0 * PI;

        double vibAmp = 0.02 * (motorRpm / 8000.0);
        vibrationNoise = Vector3(
            std::sin(vibrationPhase) * vibAmp,
            std::cos(vibrationPhase) * vibAmp,
            std::sin(2.0 * vibrationPhase) * (vibAmp * 0.5)
        );
    }

    return scaledRate + currentBias + whiteNoise + vibrationNoise;
}

} // namespace Kestrel
