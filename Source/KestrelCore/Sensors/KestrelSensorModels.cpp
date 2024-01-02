#include "KestrelSensorModels.h"
#include <algorithm>

namespace Kestrel {

KestrelSensorModels::KestrelSensorModels(const SensorSuiteConfig& config)
    : cfg(config)
    , gyroNoiseModel(config.gyroNoise, 101)
    , accelNoiseModel(config.accelNoise, 202)
    , generalNoise(AllanVarianceParams(), 303)
    , currentFixType(config.initialFixType)
    , currentHorizStdDev(config.gpsHorizStdDevMeters)
{
    Reset();
}

void KestrelSensorModels::Reset() {
    gyroNoiseModel.Reset();
    accelNoiseModel.Reset();
    generalNoise.Reset();
    currentFixType = cfg.initialFixType;
    currentHorizStdDev = cfg.gpsHorizStdDevMeters;
}

void KestrelSensorModels::SetGpsDegradation(EGpsFixType fixType, double horizStdDev) {
    currentFixType = fixType;
    currentHorizStdDev = horizStdDev;
}

void KestrelSensorModels::UpdateIMU(const RigidBodyState& trueState,
                                    double dt,
                                    double motorRpm,
                                    Vector3& outAccelBody,
                                    Vector3& outGyroBody,
                                    Vector3& outMagBody) {
    // 1. Accelerometer: specific force f_b = R^T * (a_inertial - g_inertial)
    // In NED coordinate frame: gravity is [0, 0, +9.80665] m/s^2
    Vector3 gInertial(0.0, 0.0, GRAVITY_ACCEL);
    Vector3 specificForceInertial = trueState.acceleration - gInertial;
    Vector3 trueAccelBody = trueState.attitude.RotateInverse(specificForceInertial);

    // Apply Allan variance noise, bias drift, and motor vibrations
    outAccelBody = accelNoiseModel.ApplyIMUNoise(trueAccelBody, dt, motorRpm);

    // 2. Gyroscope: true body angular velocity + Allan variance noise
    outGyroBody = gyroNoiseModel.ApplyIMUNoise(trueState.angularVelocity, dt, motorRpm);

    // 3. Magnetometer: earth field rotated into body frame + hard-iron bias + white noise
    Vector3 magBodyTrue = trueState.attitude.RotateInverse(cfg.earthMagFieldLocal);
    Vector3 magNoise(generalNoise.SampleGaussian(0.003),
                     generalNoise.SampleGaussian(0.003),
                     generalNoise.SampleGaussian(0.003));
    outMagBody = magBodyTrue + cfg.hardIronBias + magNoise;
}

void KestrelSensorModels::UpdateBarometer(const RigidBodyState& trueState,
                                         double trueAltitudeMeters,
                                         double& outPressurePa,
                                         double& outAltitudeMeters) {
    // True hydrostatic atmospheric pressure
    double trueP = KestrelAtmosphere::AltitudeToPressure(trueAltitudeMeters);

    // Aerodynamic venting dynamic pressure error (Bernoulli effect: deltaP = -0.5 * C * rho * v^2)
    double speedSq = trueState.velocity.NormSq();
    double dynamicPressure = 0.5 * AIR_DENSITY_SEA_LEVEL * speedSq;
    double bernoulliDeltaP = -cfg.venturiBernoulliFactor * dynamicPressure;

    // Measurement noise
    double noiseP = generalNoise.SampleGaussian(cfg.baroNoiseStdPa);

    outPressurePa = std::max(100.0, trueP + bernoulliDeltaP + noiseP);
    outAltitudeMeters = KestrelAtmosphere::PressureToAltitude(outPressurePa);
}

void KestrelSensorModels::UpdateGNSS(const RigidBodyState& trueState,
                                    double dt,
                                    Vector3& outGpsPos,
                                    Vector3& outGpsVel,
                                    EGpsFixType& outFixType,
                                    int& outSats) {
    (void)dt;
    outFixType = currentFixType;

    if (currentFixType == EGpsFixType::NoFix) {
        outSats = 3;
        outGpsPos = Vector3::Zero();
        outGpsVel = Vector3::Zero();
        return;
    }

    // Number of satellites based on fix quality
    if (currentFixType == EGpsFixType::RTKFixed) outSats = 18;
    else if (currentFixType == EGpsFixType::RTKFloat) outSats = 14;
    else outSats = 9;

    double horizNoiseX = generalNoise.SampleGaussian(currentHorizStdDev);
    double horizNoiseY = generalNoise.SampleGaussian(currentHorizStdDev);
    double vertNoiseZ = generalNoise.SampleGaussian(currentHorizStdDev * 2.0);

    outGpsPos = trueState.position + Vector3(horizNoiseX, horizNoiseY, vertNoiseZ);

    double velNoiseX = generalNoise.SampleGaussian(cfg.gpsVelocityStdDev);
    double velNoiseY = generalNoise.SampleGaussian(cfg.gpsVelocityStdDev);
    double velNoiseZ = generalNoise.SampleGaussian(cfg.gpsVelocityStdDev * 1.5);

    outGpsVel = trueState.velocity + Vector3(velNoiseX, velNoiseY, velNoiseZ);
}

} // namespace Kestrel
