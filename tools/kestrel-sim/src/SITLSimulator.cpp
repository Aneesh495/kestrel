#include "SITLSimulator.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>

namespace Kestrel::Sim {

SITLSimulator::SITLSimulator()
    : environmentOctree(BoundingBox3D(Vector3(-500,-500,-500), Vector3(500,500,500)))
{
    Initialize();
}

void SITLSimulator::Initialize(const Vector3& homePosition) {
    state = RigidBodyState();
    state.position = homePosition;
    state.attitude = Quat::Identity();

    simTime = 0.0;
    batteryRemaining = 100.0;
    currentPhase = EFlightPhase::Arming;

    // Define 3D inspection corridor waypoints (NED coordinates: altitude is -Z)
    waypoints = {
        Vector3(0.0, 0.0, -15.0),    // Takeoff waypoint (15m altitude)
        Vector3(25.0, 0.0, -15.0),   // Waypoint 1
        Vector3(25.0, 25.0, -15.0),  // Waypoint 2
        Vector3(0.0, 25.0, -15.0),   // Waypoint 3
        Vector3(0.0, 0.0, -15.0),    // Return home loiter point
        Vector3(0.0, 0.0, 0.0)       // Landing point
    };
    activeWaypointIndex = 0;

    posIntegral = Vector3::Zero();
    velIntegral = Vector3::Zero();
    rateIntegral = Vector3::Zero();
    lastRateError = Vector3::Zero();

    // Insert simulated building obstacle at (12, 0, -15)
    environmentOctree.Insert(1, BoundingBox3D(Vector3(10.0, -2.0, -20.0), Vector3(14.0, 2.0, -10.0)));

    telemetryLog.clear();
}

void SITLSimulator::EvaluateMissionFSM(double dt) {
    (void)dt;
    switch (currentPhase) {
        case EFlightPhase::Arming:
            if (simTime >= 1.0) {
                currentPhase = EFlightPhase::AutoTakeoff;
            }
            break;

        case EFlightPhase::AutoTakeoff:
            // Check if reached initial takeoff altitude (-15m) within 0.5m
            if (std::abs(state.position.z - waypoints[0].z) < 0.5) {
                currentPhase = EFlightPhase::WaypointFollow;
                activeWaypointIndex = 1;
            }
            break;

        case EFlightPhase::WaypointFollow: {
            Vector3 target = waypoints[activeWaypointIndex];
            double dist = (target - state.position).Norm();
            if (dist < 1.5) {
                activeWaypointIndex++;
                if (activeWaypointIndex >= waypoints.size() - 2) {
                    currentPhase = EFlightPhase::ReturnToHome;
                }
            }
            // Trigger wind gust test at t = 20s
            if (simTime >= 20.0 && simTime <= 23.0) {
                currentPhase = EFlightPhase::WindGustRecovery;
                windModel.TriggerDiscreteGust(Vector3(1.0, 0.5, 0.0), 10.0, 30.0);
            }
            break;
        }

        case EFlightPhase::WindGustRecovery:
            if (simTime > 24.0) {
                currentPhase = EFlightPhase::WaypointFollow;
            }
            break;

        case EFlightPhase::ReturnToHome: {
            Vector3 rthTarget = waypoints[waypoints.size() - 2];
            double dist = (rthTarget - state.position).Norm();
            if (dist < 1.0) {
                currentPhase = EFlightPhase::PrecisionLanding;
                activeWaypointIndex = waypoints.size() - 1;
            }
            break;
        }

        case EFlightPhase::PrecisionLanding:
            if (state.position.z >= -0.15 && state.velocity.Norm() < 0.3) {
                currentPhase = EFlightPhase::Landed;
            }
            break;

        case EFlightPhase::Landed:
        case EFlightPhase::Disarmed:
        case EFlightPhase::ObstacleAvoidance:
            break;
    }
}

MotorCommands SITLSimulator::ComputeFlightController(const Vector3& targetPos, double targetYaw, double dt) {
    MotorCommands cmds;
    if (currentPhase == EFlightPhase::Disarmed || currentPhase == EFlightPhase::Arming || currentPhase == EFlightPhase::Landed) {
        for (int i = 0; i < 4; ++i) cmds.motorPWM[i] = 0.0;
        return cmds;
    }

    // 1. Position Control Loop (P controller)
    Vector3 posError = targetPos - state.position;
    double kpPosH = 1.2;
    double kpPosZ = 1.5;
    Vector3 desVelocity(posError.x * kpPosH, posError.y * kpPosH, posError.z * kpPosZ);

    // Clamp horizontal speed to 10 m/s, vertical climb/descent to 2.5 m/s
    double horizSpeed = std::sqrt(desVelocity.x * desVelocity.x + desVelocity.y * desVelocity.y);
    if (horizSpeed > 10.0) {
        desVelocity.x = (desVelocity.x / horizSpeed) * 10.0;
        desVelocity.y = (desVelocity.y / horizSpeed) * 10.0;
    }
    desVelocity.z = std::clamp(desVelocity.z, -3.0, 2.0);

    // 2. Velocity Control Loop (PID controller)
    Vector3 velError = desVelocity - state.velocity;
    velIntegral += velError * dt;
    velIntegral.x = std::clamp(velIntegral.x, -5.0, 5.0);
    velIntegral.y = std::clamp(velIntegral.y, -5.0, 5.0);
    velIntegral.z = std::clamp(velIntegral.z, -10.0, 10.0);

    double kpVel = 2.0;
    double kiVel = 0.4;
    double kdVel = 0.15;
    Vector3 accelDemand = velError * kpVel + velIntegral * kiVel - state.acceleration * kdVel;

    // Desired roll and pitch tilt angles from horizontal accelerations
    // In NED: roll tilts positive towards +Y (East), pitch tilts negative towards +X (North)
    double desRoll = std::clamp((accelDemand.x * std::sin(targetYaw) - accelDemand.y * std::cos(targetYaw)) / GRAVITY_ACCEL, -0.45, 0.45);
    double desPitch = std::clamp((accelDemand.x * std::cos(targetYaw) + accelDemand.y * std::sin(targetYaw)) / GRAVITY_ACCEL, -0.45, 0.45);

    // Vertical collective thrust acceleration demand (compensate gravity)
    double collectiveThrustAccel = GRAVITY_ACCEL - accelDemand.z;
    double desTotalThrust = std::clamp(collectiveThrustAccel * droneParams.mass, 0.0, droneParams.mass * GRAVITY_ACCEL * 2.2);

    // 3. Attitude Control Loop
    Quat targetAttitude = Quat::FromEuler(desRoll, desPitch, targetYaw);
    Vector3 attError = KestrelQuaternionMath::ComputeAttitudeError(state.attitude, targetAttitude);

    double kpAtt = 6.5;
    Vector3 desAngularRates = attError * kpAtt;
    desAngularRates.x = std::clamp(desAngularRates.x, -4.0, 4.0);
    desAngularRates.y = std::clamp(desAngularRates.y, -4.0, 4.0);
    desAngularRates.z = std::clamp(desAngularRates.z, -2.0, 2.0);

    // 4. Rate Control Loop (PID inner loop)
    Vector3 rateError = desAngularRates - state.angularVelocity;
    rateIntegral += rateError * dt;
    rateIntegral.x = std::clamp(rateIntegral.x, -1.0, 1.0);
    rateIntegral.y = std::clamp(rateIntegral.y, -1.0, 1.0);
    rateIntegral.z = std::clamp(rateIntegral.z, -1.0, 1.0);

    Vector3 rateDerivative = (rateError - lastRateError) / dt;
    lastRateError = rateError;

    double kpRate = 0.18;
    double kiRate = 0.08;
    double kdRate = 0.005;

    Vector3 torqueDemand = rateError * kpRate + rateIntegral * kiRate + rateDerivative * kdRate;

    // 5. Quad-X Motor Mixer Matrix
    // Thrust normalized: 0.0 to 1.0
    double hoverThrottle = (droneParams.mass * GRAVITY_ACCEL) / (4.0 * droneParams.thrustCoefficient * std::pow(6000.0 * 2.0 * PI / 60.0, 2));
    double thrustNormalized = desTotalThrust / (droneParams.mass * GRAVITY_ACCEL * 2.0);

    double rollCmd = torqueDemand.x;
    double pitchCmd = torqueDemand.y;
    double yawCmd = torqueDemand.z;

    // Motors: 0: Front-Right (CCW), 1: Rear-Left (CCW), 2: Front-Left (CW), 3: Rear-Right (CW)
    cmds.motorPWM[0] = std::clamp(thrustNormalized - rollCmd + pitchCmd - yawCmd, 0.05, 1.0);
    cmds.motorPWM[1] = std::clamp(thrustNormalized + rollCmd - pitchCmd - yawCmd, 0.05, 1.0);
    cmds.motorPWM[2] = std::clamp(thrustNormalized + rollCmd + pitchCmd + yawCmd, 0.05, 1.0);
    cmds.motorPWM[3] = std::clamp(thrustNormalized - rollCmd - pitchCmd + yawCmd, 0.05, 1.0);

    return cmds;
}

void SITLSimulator::Step(double dt) {
    EvaluateMissionFSM(dt);

    Vector3 currentTarget = waypoints[activeWaypointIndex];
    double targetYaw = 0.0;

    MotorCommands motors = ComputeFlightController(currentTarget, targetYaw, dt);

    // Convert motor PWM to rotational speed and calculate thrust/torque
    double maxRadS = droneParams.maxMotorRPM * (2.0 * PI / 60.0);
    Vector3 totalBodyForce = Vector3::Zero();
    Vector3 totalBodyTorque = Vector3::Zero();

    double airDensity = KestrelAtmosphere::GetStateAtAltitude(-state.position.z).densityKgPerM3;
    Vector3 windVel = windModel.SampleWind(state.position, state.velocity.Norm(), dt);
    Vector3 relWindBody = state.attitude.RotateInverse(state.velocity - windVel);

    double motorSpins[4]{-1.0, -1.0, 1.0, 1.0}; // CCW and CW
    double arm = droneParams.armLength * 0.7071; // 45 deg Quad-X arm

    for (int i = 0; i < 4; ++i) {
        double omega = motors.motorPWM[i] * maxRadS;
        RotorState rState;
        rState.omegaRadPerS = omega;
        rState.localAirflowVelocity = relWindBody;
        rState.groundClearanceMeters = std::max(0.05, -state.position.z);

        RotorAerodynamicOutput rOut = bladeElement.ComputeRotorForces(rState, airDensity);

        // Axial thrust in body -Z direction (upward lift)
        totalBodyForce.z -= rOut.thrustNewtons;

        // Yaw torque reaction
        totalBodyTorque.z += rOut.torqueNewtonMeters * motorSpins[i];

        // Pitch & Roll moments from arm positions
        if (i == 0) { // Front-Right: (+X, +Y)
            totalBodyTorque.x -= rOut.thrustNewtons * arm;
            totalBodyTorque.y += rOut.thrustNewtons * arm;
        } else if (i == 1) { // Rear-Left: (-X, -Y)
            totalBodyTorque.x += rOut.thrustNewtons * arm;
            totalBodyTorque.y -= rOut.thrustNewtons * arm;
        } else if (i == 2) { // Front-Left: (+X, -Y)
            totalBodyTorque.x += rOut.thrustNewtons * arm;
            totalBodyTorque.y += rOut.thrustNewtons * arm;
        } else if (i == 3) { // Rear-Right: (-X, +Y)
            totalBodyTorque.x -= rOut.thrustNewtons * arm;
            totalBodyTorque.y -= rOut.thrustNewtons * arm;
        }
    }

    // Aerodynamic body drag
    Vector3 aeroDrag = relWindBody * (-droneParams.dragLinear * relWindBody.Norm());
    totalBodyForce += aeroDrag;
    totalBodyTorque -= state.angularVelocity * droneParams.dragAngular;

    // Ground contact constraint
    if (state.position.z >= 0.0 && totalBodyForce.z >= 0.0) {
        state.position.z = 0.0;
        state.velocity.z = 0.0;
        totalBodyForce.z = std::min(0.0, totalBodyForce.z);
    }

    // Runge-Kutta 4th-Order dynamical state integration
    auto dynamicsFunc = [this, totalBodyForce, totalBodyTorque](const RigidBodyState& s, double t) {
        (void)t;
        return KestrelRK4Integrator::ComputeRigidBodyDerivative(s, totalBodyForce, totalBodyTorque, droneParams);
    };

    state = KestrelRK4Integrator::Step(state, dt, dynamicsFunc, simTime);

    // Battery consumption (Coulomb counting)
    double totalPower = (motors.motorPWM[0] + motors.motorPWM[1] + motors.motorPWM[2] + motors.motorPWM[3]) * 180.0; // Watts
    double energyJoules = totalPower * dt;
    double batteryCapacityJoules = 4.0 * 3.7 * 2.2 * 3600.0; // 4S 2200mAh LiPo = ~117 kJ
    batteryRemaining -= (energyJoules / batteryCapacityJoules) * 100.0;
    batteryRemaining = std::max(0.0, batteryRemaining);

    simTime += dt;

    // Record sample
    FlightTelemetrySample sample;
    sample.timestampSeconds = simTime;
    sample.position = state.position;
    sample.velocity = state.velocity;
    double roll, pitch, yaw;
    state.attitude.ToEuler(roll, pitch, yaw);
    sample.attitudeEulerDeg = Vector3(roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
    sample.bodyRatesDegPerS = state.angularVelocity * RAD2DEG;
    sample.altitudeMslMeters = -state.position.z;
    sample.airspeedMPerS = state.velocity.Norm();
    sample.batteryPercent = batteryRemaining;
    sample.motorPWM = {motors.motorPWM[0], motors.motorPWM[1], motors.motorPWM[2], motors.motorPWM[3]};
    sample.phase = currentPhase;

    telemetryLog.push_back(sample);
}

void SITLSimulator::RunMission(double durationSeconds, double dt) {
    size_t totalSteps = static_cast<size_t>(durationSeconds / dt);
    for (size_t i = 0; i < totalSteps; ++i) {
        Step(dt);
        if (currentPhase == EFlightPhase::Landed && simTime > 35.0) {
            break;
        }
    }
}

void SITLSimulator::PrintTelemetryHUD() const {
    if (telemetryLog.empty()) return;
    const auto& s = telemetryLog.back();

    std::cout << "\n===================================================================\n";
    std::cout << "                  KESTREL AVIONICS COCKPIT HUD                    \n";
    std::cout << "===================================================================\n";
    std::cout << " TIME: " << std::fixed << std::setprecision(2) << s.timestampSeconds << "s | ALT: "
              << std::setprecision(1) << s.altitudeMslMeters << "m | IAS: "
              << std::setprecision(1) << s.airspeedMPerS << "m/s | BATT: "
              << std::setprecision(1) << s.batteryPercent << "%\n";
    std::cout << " POS (NED):  [" << std::setw(6) << s.position.x << ", "
              << std::setw(6) << s.position.y << ", "
              << std::setw(6) << s.position.z << "] m\n";
    std::cout << " ATT (R/P/Y): [" << std::setw(5) << s.attitudeEulerDeg.x << " deg, "
              << std::setw(5) << s.attitudeEulerDeg.y << " deg, "
              << std::setw(5) << s.attitudeEulerDeg.z << " deg]\n";
    std::cout << " MOTORS:     [M1: " << std::setprecision(2) << s.motorPWM[0]
              << " | M2: " << s.motorPWM[1]
              << " | M3: " << s.motorPWM[2]
              << " | M4: " << s.motorPWM[3] << "]\n";
    std::cout << " PHASE:      ";
    switch (s.phase) {
        case EFlightPhase::Arming: std::cout << "ARMING_SYSTEMS\n"; break;
        case EFlightPhase::AutoTakeoff: std::cout << "AUTO_TAKEOFF\n"; break;
        case EFlightPhase::WaypointFollow: std::cout << "WAYPOINT_NAVIGATION\n"; break;
        case EFlightPhase::WindGustRecovery: std::cout << "GUST_DISTURBANCE_REJECTION\n"; break;
        case EFlightPhase::ReturnToHome: std::cout << "RETURN_TO_HOME\n"; break;
        case EFlightPhase::PrecisionLanding: std::cout << "PRECISION_LANDING\n"; break;
        case EFlightPhase::Landed: std::cout << "TOUCHDOWN_LANDED\n"; break;
        default: std::cout << "DISARMED\n"; break;
    }
    std::cout << "===================================================================\n";
}

void SITLSimulator::ExportTelemetryCSV(const std::string& filePath) const {
    std::ofstream f(filePath);
    if (!f.is_open()) return;

    f << "time_sec,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,roll_deg,pitch_deg,yaw_deg,altitude_m,airspeed_mps,battery_pct,m1,m2,m3,m4,phase\n";
    for (const auto& s : telemetryLog) {
        f << s.timestampSeconds << ","
          << s.position.x << "," << s.position.y << "," << s.position.z << ","
          << s.velocity.x << "," << s.velocity.y << "," << s.velocity.z << ","
          << s.attitudeEulerDeg.x << "," << s.attitudeEulerDeg.y << "," << s.attitudeEulerDeg.z << ","
          << s.altitudeMslMeters << "," << s.airspeedMPerS << "," << s.batteryPercent << ","
          << s.motorPWM[0] << "," << s.motorPWM[1] << "," << s.motorPWM[2] << "," << s.motorPWM[3] << ","
          << static_cast<int>(s.phase) << "\n";
    }
}

void SITLSimulator::ExportTelemetryJSON(const std::string& filePath) const {
    std::ofstream f(filePath);
    if (!f.is_open()) return;

    f << "{\n  \"telemetry_samples\": [\n";
    for (size_t i = 0; i < telemetryLog.size(); ++i) {
        const auto& s = telemetryLog[i];
        f << "    {\"time\": " << s.timestampSeconds
          << ", \"pos\": [" << s.position.x << ", " << s.position.y << ", " << s.position.z << "]"
          << ", \"alt\": " << s.altitudeMslMeters
          << ", \"att\": [" << s.attitudeEulerDeg.x << ", " << s.attitudeEulerDeg.y << ", " << s.attitudeEulerDeg.z << "]"
          << ", \"batt\": " << s.batteryPercent << "}"
          << (i + 1 < telemetryLog.size() ? ",\n" : "\n");
    }
    f << "  ]\n}\n";
}

} // namespace Kestrel::Sim
