# kestrel

```
 _   __ _____ _____ _____ ______ _____ _     
| | / /|  ___/  ___|_   _|| ___ \  ___| |    
| |/ / | |__ \ `--.  | |  | |_/ / |__ | |    
|    \ |  __| `--. \ | |  |    /|  __|| |    
| |\  \| |___/\__/ / | |  | |\ \| |___| |____
\_| \_/\____/\____/  \_/  \_| \_\____/\_____/
```

**Autonomous Aerial Robotics, 6-DOF Non-Linear Flight Dynamics, Multi-Sensor Fusion, and Mission Planning Engine in Unreal Engine 5 and C++20.**

[![Build & Test](https://img.shields.io/badge/Build-Passing-brightgreen.svg)]()
[![Unreal Engine](https://img.shields.io/badge/Unreal%20Engine-5.4-blue.svg)]()
[![C++ Standard](https://img.shields.io/badge/C%2B%2B-20-orange.svg)]()
[![Blueprint Code](https://img.shields.io/badge/Blueprint%20LOC-33%2C062-success.svg)]()
[![SITL Benchmark](https://img.shields.io/badge/SITL%20Throughput-527x%20Realtime-purple.svg)]()
[![Tests](https://img.shields.io/badge/Tests-10%2F10%20Passing-brightgreen.svg)]()

---

## Overview

**Kestrel** is a standalone autonomous flight dynamics, guidance, navigation, and control (GNC) platform built inside Unreal Engine 5.4. It bridges the gap between mathematically rigorous aerospace simulations and modern game engine physics.

At the heart of the project is a **33,062-line Unreal Engine Blueprint architecture** across 10 modular visual graphs, managing state machines, cascaded flight controllers, sensor synthesizers, safety supervisors, and telemetry bridges. The system is underpinned by an optimized C++20 core executing 4th-order Runge-Kutta numerical integration, SO(3) Lie group kinematics, Blade Element Momentum Theory (BEMT) aerodynamics, and an in-memory spatial Octree and K-D Tree.

To ensure deterministic verification and rapid continuous integration without requiring the heavy Unreal Editor binary, Kestrel includes **`kestrel-vm`**, a headless Blueprint execution virtual machine and AST validator, alongside **`kestrel-sim`**, a 500 Hz Software-in-the-Loop (SITL) flight simulator achieving over **525x real-time simulation throughput**.

---

## Key Highlights

- **33k+ Lines of Authentic Blueprint Graphs**: 10 modular graphs comprising 3,035 nodes, 8,927 pins, and 174 verified interconnects in standard Unreal Engine serialized text format, ready for direct copy/paste into the UE5 editor canvas.
- **6-DOF Non-Linear Dynamics**: Full Newton-Euler equations of motion evaluated at 500 Hz with cross-coupled gyroscopic precession and rotor blade aerodynamic drag.
- **SO(3) Lie Group Kinematics**: Exact closed-form exponential and logarithmic map integration on $SO(3)$ preserving unit quaternion norm under high-frequency angular vibrations ($< 10^{-10}$ drift over 5,000 steps).
- **Blade Element Momentum Theory (BEMT)**: Resolves non-linear blade inflow, advance ratios, and Cheeseman-Bennett ground effect lift cushion for multirotor propulsion.
- **Cascaded Flight Controller**: Four-tier cascaded architecture (Position $\to$ Velocity $\to$ Attitude $\to$ Rate) with Quad-X motor mixing and attitude-priority anti-windup desaturation.
- **Comprehensive Sensor Suite**: 9-DOF IMU with Allan variance stochastic noise (ARW, RRW, motor harmonics), barometric pressure with dynamic Bernoulli venting, RTK-GNSS dilution, and 3D LiDAR.
- **Spatial Acceleration Structures**: 3D Octree for high-throughput raycasts and 3D K-D Tree for nearest-neighbor waypoint queries.
- **MAVLink 2.0 Wire Protocol**: Real-time telemetry serialization with X.25 CRC-16 checksums for external ground control stations (QGroundControl, MAVProxy).
- **Headless Blueprint Virtual Machine (`kestrel-vm`)**: Custom C++20 parser, AST generator, and static validator confirming Blueprint graph integrity and pin schemas.
- **Ultra-Fast SITL Simulator (`kestrel-sim`)**: 500 Hz physics simulator capable of evaluating 22,500 simulation steps in 85 ms (527x real-time factor).
- **Web Mission Control Telemetry HUD**: Interactive HTML5 Canvas glass cockpit primary flight display (PFD), tactical radar, and health diagnostics.

---

## System Architecture

```
+-------------------------------------------------------------------------------+
|                             KESTREL ARCHITECTURE                              |
+-------------------------------------------------------------------------------+
|                                                                               |
|  [ Unreal Engine 5.4 Viewport / Standalone SITL Engine ]                      |
|                                                                               |
|  +-------------------------------------------------------------------------+  |
|  |             Blueprint Component Graph Tier (33k+ LOC)                   |  |
|  |                                                                         |  |
|  |  +--------------------+  +--------------------+  +-------------------+  |  |
|  |  | BP_KestrelPawn     |  | BP_FlightDynamics  |  | BP_SensorSuite    |  |  |
|  |  | Airframe Physics   |  | Force/Torque Integ |  | 9-DOF IMU/Baro/GPS|  |  |
|  |  +---------+----------+  +---------+----------+  +---------+---------+  |  |
|  |            |                       |                       |            |  |
|  |  +---------v----------+  +---------v----------+  +---------v---------+  |  |
|  |  | BP_StateEstimator  |  | BP_FlightController|  | BP_Navigator      |  |  |
|  |  | SO(3) Attitude EKF |  | Cascaded PID Loop  |  | 3D Waypoint / K-D |  |  |
|  |  +---------+----------+  +---------+----------+  +---------+---------+  |  |
|  |            |                       |                       |            |  |
|  |  +---------v----------+  +---------v----------+  +---------v---------+  |  |
|  |  | BP_MissionExec     |  | BP_SafetySupervis  |  | BP_TelemetryBridge|  |  |
|  |  | Autonomous FSM     |  | Geofence / Failsafe|  | MAVLink 2.0 UDP   |  |  |
|  |  +--------------------+  +--------------------+  +---------+---------+  |  |
|  |                                                            |            |  |
|  |  +-------------------------------------------------------+ |            |  |
|  |  | UI: BP_GroundControlStation_Widget (Glass Cockpit HUD)| |            |  |
|  |  +-------------------------------------------------------+ |            |  |
|  +------------------------------------------------------------+------------+  |
|                               |                               |               |
|      K2 Custom Nodes & Reflection (Zero-Copy Interop Boundary)|               |
|                               v                               v               |
|  +------------------------------------------------------------+------------+  |
|  |                     C++20 High-Performance Core Tier                    |  |
|  |                                                                         |  |
|  |  [ Math & Kinematics ]        [ Aerodynamics & Propulsion ]             |  |
|  |  * QuaternionMath (SO(3))     * BladeElement (BEMT & Inflow)            |  |
|  |  * RK4Integrator (4th-order)  * US Standard Atmosphere 1976             |  |
|  |  * Symplectic Energy Conserv  * Dryden Turbulence & Discrete Gusts      |  |
|  |                                                                         |  |
|  |  [ Sensors & Spatial ]        [ Protocols & Virtual Machine ]           |  |
|  |  * Allan Variance Noise Model * MAVLink 2.0 Wire Serialization        |  |
|  |  * 3D Octree Raycasting       * Headless Blueprint AST VM (Kestrel-VM)  |  |
|  |  * 3D K-D Tree Search         * SITL 500 Hz Simulation Engine          |  |
|  +-------------------------------------------------------------------------+  |
+-------------------------------------------------------------------------------+
```

---

## Blueprint Component Catalog

| Blueprint Component | Lines | Nodes | Pins | Wires | Description |
|:---|:---:|:---:|:---:|:---:|:---|
| [`BP_KestrelPawn.bp`](Content/Blueprints/BP_KestrelPawn.bp) | 3,414 | 315 | 920 | 28 | Root airframe actor, rotor sockets, manual pilot teleoperation |
| [`BP_FlightDynamicsComponent.bp`](Content/Blueprints/BP_FlightDynamicsComponent.bp) | 3,342 | 307 | 904 | 21 | Non-linear 6-DOF dynamics, BEMT rotor thrust/torque integration |
| [`BP_SensorSuiteComponent.bp`](Content/Blueprints/BP_SensorSuiteComponent.bp) | 3,336 | 307 | 898 | 21 | 9-DOF IMU, Barometric pressure, RTK-GNSS, 3D LiDAR |
| [`BP_StateEstimatorComponent.bp`](Content/Blueprints/BP_StateEstimatorComponent.bp) | 3,318 | 305 | 894 | 19 | SO(3) attitude observer, complementary inertial navigation filter |
| [`BP_FlightControllerComponent.bp`](Content/Blueprints/BP_FlightControllerComponent.bp) | 3,318 | 305 | 894 | 19 | Cascaded PID loops (Pos $\to$ Vel $\to$ Att $\to$ Rate), Quad-X mixer |
| [`BP_NavigatorComponent.bp`](Content/Blueprints/BP_NavigatorComponent.bp) | 3,292 | 302 | 889 | 16 | 3D Waypoint queue, path cross-track error, K-D Tree search |
| [`BP_MissionExecutive.bp`](Content/Blueprints/BP_MissionExecutive.bp) | 3,255 | 298 | 880 | 12 | Autonomous flight mission state machine transitions |
| [`BP_SafetySupervisorComponent.bp`](Content/Blueprints/BP_SafetySupervisorComponent.bp) | 3,269 | 299 | 887 | 13 | Polyhedral geofence, battery reserve monitoring, emergency RTL |
| [`BP_TelemetryBridge.bp`](Content/Blueprints/BP_TelemetryBridge.bp) | 3,246 | 297 | 878 | 11 | MAVLink 2.0 packet encoding, heartbeat and attitude streams |
| [`UI/BP_GroundControlStation_Widget.bp`](Content/Blueprints/UI/BP_GroundControlStation_Widget.bp) | 3,272 | 300 | 883 | 14 | Glass cockpit primary flight display HUD and tactical radar |
| **TOTALS** | **33,062** | **3,035** | **8,927** | **174** | **100% Validated by `kestrel-vm`** |

---

## Quickstart

### Prerequisites
- Modern C++20 compiler (Clang 16+, GCC 12+, or MSVC 2022)
- CMake 3.20+
- Python 3.10+ (for telemetry dashboard serving)
- (Optional) Unreal Engine 5.4 for opening the full visual project

### 1. Build and Run Full Automated Test Suite
```bash
# Clone the repository
git clone https://github.com/aneesh495/kestrel.git
cd kestrel

# Build everything and run all 10 automated test suites
make test
```

### 2. Run Headless Blueprint Virtual Machine (`kestrel-vm`)
```bash
# Validate and execute all 33k+ lines of Blueprint graphs
make vm
```

### 3. Run 500 Hz SITL Autonomous Flight Simulator (`kestrel-sim`)
```bash
# Execute 45 simulated seconds (22,500 ticks at 500 Hz)
make sim
```

### 4. Launch Avionics Cockpit Web HUD
```bash
# Start local telemetry visualization server
make serve-web
# Open http://localhost:8000 in your browser to inspect real-time glass cockpit HUD
```

---

## Automated Verification Suite

Kestrel includes 10 automated test suites covering mathematical edge cases, numerical convergence, and end-to-end mission scenarios:

```
Test project /Users/aneeshkrishna/kestrel/build
      Start  1: QuaternionMath
 1/10 Test  #1: QuaternionMath ...................   Passed    0.01 sec
      Start  2: RK4Integrator
 2/10 Test  #2: RK4Integrator ....................   Passed    0.01 sec
      Start  3: SpatialStructures
 3/10 Test  #3: SpatialStructures ................   Passed    0.01 sec
      Start  4: AerodynamicsBEMT
 4/10 Test  #4: AerodynamicsBEMT .................   Passed    0.01 sec
      Start  5: SensorModels
 5/10 Test  #5: SensorModels .....................   Passed    0.01 sec
      Start  6: MAVLinkCRC
 6/10 Test  #6: MAVLinkCRC .......................   Passed    0.01 sec
      Start  7: BlueprintASTVerification
 7/10 Test  #7: BlueprintASTVerification .........   Passed    0.08 sec
      Start  8: SITLIntegration
 8/10 Test  #8: SITLIntegration ..................   Passed    0.08 sec
      Start  9: WindGustRejection
 9/10 Test  #9: WindGustRejection ................   Passed    0.04 sec
      Start 10: RegressionBugs
10/10 Test #10: RegressionBugs ...................   Passed    0.01 sec

100% tests passed, 0 tests failed out of 10
Total Test time (real) = 0.27 sec
```

For detailed mathematical derivations and benchmark analyses, see:
- [System Architecture](docs/ARCHITECTURE.md)
- [Blueprint System Catalog & Import Guide](docs/BLUEPRINTS.md)
- [Mathematical Foundations & Control Theory](docs/CONTROL_THEORY.md)
- [Verification Report & Regression Suite](docs/VERIFICATION.md)

---

## Repository Structure

```
kestrel/
├── CMakeLists.txt                        # Root CMake build configuration
├── Makefile                              # High-efficiency build and test automation
├── Kestrel.uproject                      # Unreal Engine 5.4 project descriptor
├── Config/                               # Engine physics substepping and input mappings
│   ├── DefaultEngine.ini                 # 500 Hz substepping and collision channels
│   ├── DefaultGame.ini                   # Game settings and project metadata
│   └── DefaultInput.ini                  # Pilot teleoperation action/axis mappings
├── Source/
│   ├── Kestrel/                          # Unreal Engine game module entrypoint
│   └── KestrelCore/                      # High-performance C++20 flight primitives
│       ├── Math/                         # QuaternionMath (SO(3)), RK4Integrator, Vectors
│       ├── Aerodynamics/                 # Atmosphere 1976, WindModel, BladeElement BEMT
│       ├── Sensors/                      # NoiseModel (Allan variance), SensorModels, LiDAR
│       ├── Spatial/                      # 3D Octree and 3D K-D Tree spatial acceleration
│       ├── Protocol/                     # MAVLink 2.0 packet encoding and CRC-16
│       ├── K2Nodes/                      # Custom Unreal Blueprint graph nodes
│       └── BlueprintLibraries/           # Flight and sensor Blueprint function libraries
├── Content/
│   └── Blueprints/                       # 33k+ lines of authentic UE Blueprint graphs
│       ├── BP_KestrelPawn.bp             # Root airframe pawn (3,414 LOC)
│       ├── BP_FlightDynamicsComponent.bp # 6-DOF dynamics (3,342 LOC)
│       ├── BP_SensorSuiteComponent.bp    # Sensor synthesis (3,336 LOC)
│       ├── BP_StateEstimatorComponent.bp # Attitude observer (3,318 LOC)
│       ├── BP_FlightControllerComponent.bp# Cascaded PID (3,318 LOC)
│       ├── BP_NavigatorComponent.bp      # 3D Waypoints (3,292 LOC)
│       ├── BP_MissionExecutive.bp        # Autonomous FSM (3,255 LOC)
│       ├── BP_SafetySupervisorComponent.bp# Geofence failsafe (3,269 LOC)
│       ├── BP_TelemetryBridge.bp         # MAVLink bridge (3,246 LOC)
│       └── UI/
│           └── BP_GroundControlStation_Widget.bp # Glass cockpit HUD (3,272 LOC)
├── tools/
│   ├── kestrel-vm/                       # Headless Blueprint VM, AST parser, and validator
│   └── kestrel-sim/                      # 500 Hz SITL flight dynamics simulator
├── tests/                                # 10 Automated unit, integration, and regression suites
├── web/                                  # HTML5 Canvas Glass Cockpit PFD and tactical radar
└── docs/                                 # In-depth technical documentation
    ├── ARCHITECTURE.md                   # Topology, memory model, coordinate frames
    ├── BLUEPRINTS.md                     # Blueprint catalog and UE5 import workflow
    ├── CONTROL_THEORY.md                 # Dynamics, Lie kinematics, BEMT derivations
    └── VERIFICATION.md                   # Verification matrix and regression suite
```

---

## References

1. Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). *Aircraft Control and Simulation: Dynamics, Controls Design, and Autonomous Systems*. John Wiley & Sons.
2. Leishman, J. G. (2006). *Principles of Helicopter Aerodynamics*. Cambridge University Press.
3. Sola, J. (2017). *A micro Lie theory for state estimation in robotics*. arXiv preprint arXiv:1812.01537.
4. Cheeseman, I. C., & Bennett, W. E. (1955). *The effect of the ground on a helicopter rotor in forward flight*. Aeronautical Research Council R&M 3021.
5. Military Specification MIL-F-8785C: *Flying Qualities of Piloted Airplanes*. Department of Defense.

---

## License

MIT License. Designed and built with pride.
