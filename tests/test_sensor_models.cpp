#include "Sensors/KestrelSensorModels.h"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace Kestrel;

void TestStationaryAccelerometerSpecificForce() {
    KestrelSensorModels sensors;
    RigidBodyState stationaryState;
    stationaryState.position = Vector3(0, 0, 0);
    stationaryState.velocity = Vector3(0, 0, 0);
    stationaryState.acceleration = Vector3(0, 0, 0);
    stationaryState.attitude = Quat::Identity(); // Level attitude

    Vector3 accelBody, gyroBody, magBody;
    sensors.UpdateIMU(stationaryState, 0.002, 0.0, accelBody, gyroBody, magBody);

    // Stationary drone on earth experiences -g specific force in NED frame (i.e. -9.80665 m/s^2 on Z)
    assert(std::abs(accelBody.z - (-GRAVITY_ACCEL)) < 0.2); // Within noise tolerance
    assert(std::abs(accelBody.x) < 0.2);
    assert(std::abs(accelBody.y) < 0.2);

    // Gyro rates should be near zero (only sensor noise and small bias)
    assert(gyroBody.Norm() < 0.05);

    std::cout << "  [PASS] Accelerometer Specific Force and Stationary 1g Reaction\n";
}

void TestBarometerBernoulliDynamicPressure() {
    KestrelSensorModels sensors;
    RigidBodyState movingState;
    movingState.position = Vector3(0, 0, -20.0); // 20m altitude
    movingState.velocity = Vector3(15.0, 0.0, 0.0); // 15 m/s horizontal flight

    double pressurePa = 0.0;
    double altitudeMeters = 0.0;
    sensors.UpdateBarometer(movingState, 20.0, pressurePa, altitudeMeters);

    // Dynamic venting lowers measured static pressure via Bernoulli effect
    // Lower pressure inverts to slightly higher apparent altitude
    assert(altitudeMeters >= 19.0 && altitudeMeters <= 22.0);
    assert(pressurePa < 101325.0);

    std::cout << "  [PASS] Barometer Venting Correction and Bernoulli Dynamic Pressure\n";
}

void TestGNSSFixDegradationModes() {
    KestrelSensorModels sensors;
    RigidBodyState state;
    state.position = Vector3(50.0, 100.0, -15.0);

    Vector3 gpsPos, gpsVel;
    EGpsFixType fixType;
    int sats;

    // Normal RTK Fixed mode
    sensors.UpdateGNSS(state, 0.1, gpsPos, gpsVel, fixType, sats);
    assert(fixType == EGpsFixType::RTKFixed);
    assert(sats >= 16);
    assert((gpsPos - state.position).Norm() < 0.15); // Sub-decimeter precision

    // Simulate GPS degradation / jamming
    sensors.SetGpsDegradation(EGpsFixType::Fix3D, 2.5); // Standard standalone 3D fix
    sensors.UpdateGNSS(state, 0.1, gpsPos, gpsVel, fixType, sats);
    assert(fixType == EGpsFixType::Fix3D);
    assert(sats <= 12);

    std::cout << "  [PASS] RTK-GNSS Satellite DOP and Degradation Transitions\n";
}

int main() {
    std::cout << "=== KESTREL TEST SUITE: SENSOR SUITE & NOISE MODELS ===\n";
    TestStationaryAccelerometerSpecificForce();
    TestBarometerBernoulliDynamicPressure();
    TestGNSSFixDegradationModes();
    std::cout << "ALL SENSOR SUITE TESTS PASSED.\n\n";
    return 0;
}
