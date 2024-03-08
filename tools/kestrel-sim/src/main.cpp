#include "SITLSimulator.h"
#include <iostream>
#include <chrono>

using namespace Kestrel::Sim;

int main(int argc, char** argv) {
    std::cout << "===================================================================\n";
    std::cout << " KESTREL SOFTWARE-IN-THE-LOOP (SITL) AUTONOMOUS FLIGHT SIMULATOR\n";
    std::cout << " 500 Hz 6-DOF Dynamics | BEMT Rotor Aerodynamics | Dryden Turbulence\n";
    std::cout << "===================================================================\n";

    double missionDuration = 45.0; // seconds
    if (argc > 1) {
        try { missionDuration = std::stod(argv[1]); } catch (...) {}
    }

    std::cout << "Initializing Autonomous Inspection Mission (" << missionDuration << " simulated seconds)...\n";

    SITLSimulator sim;
    sim.Initialize();

    auto startTime = std::chrono::high_resolution_clock::now();

    // Step simulation at 500 Hz (dt = 0.002s)
    double dt = 0.002;
    size_t totalTicks = static_cast<size_t>(missionDuration / dt);

    std::cout << "Executing " << totalTicks << " physics and control loop ticks at 500 Hz...\n";

    for (size_t tick = 0; tick < totalTicks; ++tick) {
        sim.Step(dt);

        // Print HUD update every 5 seconds (2500 ticks)
        if (tick % 2500 == 0 || tick == totalTicks - 1) {
            sim.PrintTelemetryHUD();
        }

        if (sim.GetPhase() == EFlightPhase::Landed && tick > 15000) {
            std::cout << "\nAutonomous Mission Complete: Vehicle touched down and landed safely.\n";
            sim.PrintTelemetryHUD();
            break;
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    double wallTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    std::cout << "\n-------------------------------------------------------------------\n";
    std::cout << " SIMULATION BENCHMARK SUMMARY\n";
    std::cout << " Total Telemetry Samples Recorded: " << sim.GetHistory().size() << "\n";
    std::cout << " Wall Clock Execution Time:        " << wallTimeMs << " ms\n";
    std::cout << " Real-Time Factor (RTF):           " << (sim.GetState().position.z != 0.0 ? (sim.GetHistory().size() * dt * 1000.0) / wallTimeMs : 0.0) << "x speed of light\n";
    std::cout << " Exporting Telemetry Logs (telemetry.csv, telemetry.json)... ";

    sim.ExportTelemetryCSV("telemetry.csv");
    sim.ExportTelemetryJSON("telemetry.json");
    std::cout << "DONE.\n";
    std::cout << "-------------------------------------------------------------------\n";

    return 0;
}
