#pragma once

#include "Math/KestrelMathTypes.h"
#include "Math/KestrelQuaternionMath.h"
#include "Math/KestrelRK4Integrator.h"
#include "Aerodynamics/KestrelAtmosphere.h"
#include "Aerodynamics/KestrelWindModel.h"
#include "Aerodynamics/KestrelBladeElement.h"
#include "Sensors/KestrelSensorModels.h"
#include "Sensors/KestrelLiDARScanner.h"
#include "Spatial/KestrelOctree.h"
#include "Spatial/KestrelKDTree.h"
#include "Protocol/KestrelMAVLink.h"
#include <vector>
#include <string>

namespace Kestrel::Sim {

enum class EFlightPhase {
    Disarmed,
    Arming,
    AutoTakeoff,
    WaypointFollow,
    ObstacleAvoidance,
    WindGustRecovery,
    ReturnToHome,
    PrecisionLanding,
    Landed
};

struct FlightTelemetrySample {
    double timestampSeconds{0.0};
    Vector3 position{0.0, 0.0, 0.0};
    Vector3 velocity{0.0, 0.0, 0.0};
    Vector3 attitudeEulerDeg{0.0, 0.0, 0.0};
    Vector3 bodyRatesDegPerS{0.0, 0.0, 0.0};
    double altitudeMslMeters{0.0};
    double airspeedMPerS{0.0};
    double batteryPercent{100.0};
    std::array<double, 4> motorPWM{0.0, 0.0, 0.0, 0.0};
    EFlightPhase phase{EFlightPhase::Disarmed};
};

/**
 * 500 Hz Software-in-the-Loop (SITL) Autonomous Flight Simulator.
 * Integrates 6-DOF dynamics, BEMT propulsion, Dryden atmospheric turbulence,
 * cascaded PID attitude control, and autonomous mission waypoint sequencing.
 */
class SITLSimulator {
public:
    SITLSimulator();

    void Initialize(const Vector3& homePosition = Vector3::Zero());
    void Step(double dt);
    void RunMission(double durationSeconds, double dt = 0.002);

    const RigidBodyState& GetState() const { return state; }
    EFlightPhase GetPhase() const { return currentPhase; }
    const std::vector<FlightTelemetrySample>& GetHistory() const { return telemetryLog; }

    void ExportTelemetryCSV(const std::string& filePath) const;
    void ExportTelemetryJSON(const std::string& filePath) const;
    void PrintTelemetryHUD() const;

private:
    RigidBodyState state;
    DronePhysicalParams droneParams;
    KestrelWindModel windModel;
    KestrelBladeElement bladeElement;
    KestrelSensorModels sensorSuite;
    KestrelOctree environmentOctree;

    EFlightPhase currentPhase{EFlightPhase::Disarmed};
    double simTime{0.0};
    double batteryRemaining{100.0};

    // Waypoint mission plan
    std::vector<Vector3> waypoints;
    size_t activeWaypointIndex{0};

    // Controller states
    Vector3 posIntegral{0.0, 0.0, 0.0};
    Vector3 velIntegral{0.0, 0.0, 0.0};
    Vector3 rateIntegral{0.0, 0.0, 0.0};
    Vector3 lastRateError{0.0, 0.0, 0.0};

    std::vector<FlightTelemetrySample> telemetryLog;

    MotorCommands ComputeFlightController(const Vector3& targetPos, double targetYaw, double dt);
    void EvaluateMissionFSM(double dt);
};

} // namespace Kestrel::Sim
