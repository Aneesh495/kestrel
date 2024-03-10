#pragma once

#include "Math/KestrelMathTypes.h"
#include "KestrelNoiseModel.h"
#include "Aerodynamics/KestrelAtmosphere.h"

namespace Kestrel {

enum class EGpsFixType : uint8_t {
    NoFix = 0,
    Fix2D = 1,
    Fix3D = 2,
    DGPS = 3,
    RTKFloat = 4,
    RTKFixed = 5
};

struct SensorSuiteConfig {
    AllanVarianceParams gyroNoise;
    AllanVarianceParams accelNoise;
    Vector3 earthMagFieldLocal{0.22, 0.05, 0.42}; // Local geomagnetic field vector (Gauss)
    Vector3 hardIronBias{0.015, -0.02, 0.005};    // Ferromagnetic hard-iron distortion
    double baroNoiseStdPa{1.5};                   // Barometric pressure white noise (Pa)
    double venturiBernoulliFactor{0.05};          // Dynamic pressure venting coefficient
    double gpsHorizStdDevMeters{0.02};            // RTK-Fixed horizontal precision (m)
    double gpsVertStdDevMeters{0.05};             // RTK-Fixed vertical precision (m)
    double gpsVelocityStdDev{0.03};               // Velocity measurement precision (m/s)
    EGpsFixType initialFixType{EGpsFixType::RTKFixed};
};

/**
 * Multi-Sensor Simulation Suite.
 * Fuses 9-DOF IMU specific force and angular rate measurements,
 * geomagnetic field rotations, Bernoulli dynamic pressure barometric corrections,
 * and RTK-GNSS satellite dilution of precision (PDOP).
 */
class KestrelSensorModels {
public:
    KestrelSensorModels();
    explicit KestrelSensorModels(const SensorSuiteConfig& config);

    void Reset();

    /**
     * Synthesizes 9-DOF IMU accelerometer specific force and gyroscope rates.
     */
    void UpdateIMU(const RigidBodyState& trueState,
                   double dt,
                   double motorRpm,
                   Vector3& outAccelBody,
                   Vector3& outGyroBody,
                   Vector3& outMagBody);

    /**
     * Synthesizes barometric static pressure and altitude with dynamic aerodynamic venting.
     */
    void UpdateBarometer(const RigidBodyState& trueState,
                         double trueAltitudeMeters,
                         double& outPressurePa,
                         double& outAltitudeMeters);

    /**
     * Synthesizes RTK-GNSS position and velocity telemetry.
     */
    void UpdateGNSS(const RigidBodyState& trueState,
                    double dt,
                    Vector3& outGpsPos,
                    Vector3& outGpsVel,
                    EGpsFixType& outFixType,
                    int& outSats);

    void SetGpsDegradation(EGpsFixType fixType, double horizStdDev);

private:
    SensorSuiteConfig cfg;
    KestrelNoiseModel gyroNoiseModel;
    KestrelNoiseModel accelNoiseModel;
    KestrelNoiseModel generalNoise;

    EGpsFixType currentFixType;
    double currentHorizStdDev;
};

} // namespace Kestrel
