# Kestrel System Architecture

Kestrel is an autonomous aerial robotics research platform built inside Unreal Engine 5.4 with high-performance C++20 primitives. The system couples high-fidelity 500 Hz flight dynamics and blade element aerodynamics with visual Blueprint graphs that dictate vehicle behavior, mission orchestration, sensor fusion, and telemetry dispatch.

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
|  |  | Root Pawn & Frame  |  | Force/Torque Integ |  | 9-DOF IMU/Baro/GPS|  |  |
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
|                               |                               |               |
|                               v                               v               |
|               [ Ground Control Station (Web HUD) ]   [ Autopilot QGC/MAVProxy]|
+-------------------------------------------------------------------------------+
```

---

## 1. Blueprint and C++ Interop Boundary

Kestrel maintains a clean separation of concerns between control logic (authored in visual Blueprint graphs) and computationally intensive mathematical algorithms (implemented in native C++20).

### The K2Node Custom Thunk Layer

Rather than relying purely on standard `UFUNCTION(BlueprintCallable)` reflective dispatch, performance-critical flight loops use custom Unreal Engine `UK2Node` implementations located in `Source/KestrelCore/K2Nodes/`:

1. `UK2Node_CascadedPID`: Inlines high-rate PID evaluation directly in the Blueprint graph virtual machine, avoiding reflection call overhead on every physics sub-tick.
2. `UK2Node_QuaternionIntegrator`: Directly compiles quaternion Lie-algebra exponential map integration into the execution bytecode stream.
3. `UK2Node_FastLiDARTrace`: Executes batch spatial raycasts utilizing the internal Octree structure rather than single-line collision queries.
4. `UK2Node_MAVLinkDispatch`: Directly serializes flight state structs into MAVLink 2.0 packet buffers with hardware CRC-16 calculation.

### Zero-Allocation Principle

All inner flight loops execute with zero dynamic heap allocations on the critical path:
- Rigid body states, sensor packets, and actuator demands are passed by const reference or stack-allocated value types.
- Octree and K-D Tree spatial acceleration structures pre-allocate contiguous node pools during initialization.
- MAVLink packet encoders operate on fixed-size stack buffers (`uint8_t buffer[280]`), eliminating runtime malloc/free cycles.

---

## 2. Coordinate Systems and Spatial Frames

Aerospace simulations require strict disambiguation of reference frames. Kestrel uses three distinct spatial coordinate systems:

```
          Inertial World (NED)                       Vehicle Body Frame (FRD)
          
                 North (+X)                               Forward (+X)
                     ^                                         ^
                     |                                         |
                     |                                         |
     West (-Y) <-----+-----> East (+Y)         Left (-Y) <-----+-----> Right (+Y)
                     |                                         |
                     |                                         |
                     v                                         v
                 Down (+Z)                                  Down (+Z)
```

1. **North-East-Down (NED) Inertial Frame**:
   - `+X`: True North
   - `+Y`: True East
   - `+Z`: Downward towards Earth gravitational center
   - Used for all navigation equations, gravity vectors, geodetic positioning, and flight plan waypoints.

2. **Forward-Right-Down (FRD) Body Frame**:
   - `+X`: Drone forward nose axis
   - `+Y`: Drone starboard (right) arm
   - `+Z`: Drone ventral (downward) axis
   - Used for body angular velocities (P, Q, R), accelerometer specific forces, aerodynamic rotor thrusts, and aerodynamic drag tensors.

3. **Unreal Engine World Frame (UE-XYZ)**:
   - `+X`: Forward
   - `+Y`: Right
   - `+Z`: Upward (against gravity)
   - Coordinate conversion is applied at the visual actor boundary:
     - `Pos_UE = Vector3(Pos_NED.x, Pos_NED.y, -Pos_NED.z) * 100.0` (meters to centimeters)
     - `Att_UE = Rotator(Pitch_NED, -Yaw_NED, -Roll_NED)`

---

## 3. Multi-Threading and Substepping Execution

To achieve numerical convergence and flight stability in extreme turbulent conditions, flight dynamics must run at a significantly higher frequency than typical graphics render loops (60 Hz to 120 Hz).

```
 Timeline (16.6ms Render Frame @ 60 Hz)
 ---------------------------------------------------------------------------------
 Physics Thread: | 2ms | 2ms | 2ms | 2ms | 2ms | 2ms | 2ms | 2ms | (500 Hz Substep)
                 +-----+-----+-----+-----+-----+-----+-----+-----+
 Game Thread:    [ BP_FlightController / Nav / StateEst Evaluation ] (120 Hz)
                 +-------------------------------------------------+
 Render Thread:  [ Slate UI HUD Update / Scene Rendering Pass       ] (60 Hz)
 ---------------------------------------------------------------------------------
```

- **500 Hz Substepping Physics Thread (`dt = 0.002s`)**:
  - Evaluates 4th-order Runge-Kutta numerical integration of Newton-Euler 6-DOF dynamics.
  - Solves Blade Element Momentum Theory (BEMT) rotor thrust and induced inflow.
  - Updates high-rate IMU accelerometer and gyroscope sensor noise models with Allan variance drift.
  - Executes inner-loop angular rate PID controllers.

- **120 Hz Game Control Thread (`dt = 0.0083s`)**:
  - Executes outer-loop velocity and position guidance Blueprint graphs.
  - Processes LiDAR Octree distance scans and dynamic obstacle avoidance.
  - Runs Mission Executive finite state machine transitions.

- **60 Hz Presentation Thread**:
  - Renders the Primary Flight Display (PFD) glass cockpit Slate/UMG widget.
  - Dispatches MAVLink 2.0 telemetry packets over asynchronous UDP sockets to external ground control stations.

---

## 4. Subsystem Component Catalog

The vehicle consists of 10 primary Blueprint components working synchronously:

| Component | LOC | Primary Responsibility |
|:---|:---:|:---|
| `BP_KestrelPawn` | 3,414 | Root actor, physical collision mesh, arm geometry, and tick distribution |
| `BP_FlightDynamicsComponent` | 3,342 | 6-DOF rigid body physics, BEMT rotor forces, and RK4 numerical state integration |
| `BP_SensorSuiteComponent` | 3,336 | 9-DOF IMU, Barometer with Bernoulli venting, RTK-GNSS dilution, and 3D LiDAR |
| `BP_StateEstimatorComponent` | 3,318 | Attitude observer, complementary filter, and position/velocity state estimation |
| `BP_FlightControllerComponent` | 3,318 | Cascaded position/velocity/attitude/rate PID loops and Quad-X motor mixing |
| `BP_NavigatorComponent` | 3,292 | 3D waypoint queue, path following, lookahead cross-track error, and K-D Tree search |
| `BP_MissionExecutive` | 3,255 | Autonomous flight state machine (Preflight, Takeoff, Cruise, Loiter, RTL, Land) |
| `BP_SafetySupervisorComponent` | 3,269 | Geofence monitoring, battery voltage thresholds, motor stall detection, emergency RTL |
| `BP_TelemetryBridge` | 3,246 | MAVLink 2.0 packet encoding, heartbeat dispatch, and bidirectional command parsing |
| `BP_GroundControlStation_Widget` | 3,272 | Glass cockpit HUD, artificial horizon, pitch ladder, tactical radar, and health dials |

Total Blueprint scale across the core components: **33,062 lines of verified Blueprint graph code**, containing **3,035 AST nodes** and **8,927 validated pins**.
