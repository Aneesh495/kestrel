# Unreal Engine Blueprint System Catalog

Kestrel features an authentic, enterprise-scale Unreal Engine Blueprint architecture comprising **33,062 lines of Blueprint code** across 10 specialized graph modules. Every graph is authored in standard Unreal Engine serialized text format, rendering them fully compatible with both the Unreal Engine 5.4 Blueprint visual graph canvas and the standalone `kestrel-vm` headless execution engine.

---

## 1. Unreal Engine Blueprint Text Format Specification

In Unreal Engine, any node graph copied to the clipboard or exported from the editor is serialized as structured ASCII text adhering to the `K2Node` object model. Each node definition follows this grammar:

```
Begin Object Class=/Script/BlueprintGraph.K2Node_<NodeType> Name="<Node_GUID>"
   <Property>=<Value>
   CustomProperties Pin (PinId=<GUID>,PinName="<Name>",Direction="<EGPD_Input|EGPD_Output>",PinType.PinCategory="<Category>",LinkedTo=(...))
   NodePosX=<X>
   NodePosY=<Y>
End Object
```

### Visual Editor Copy/Paste Workflow

To import any Kestrel Blueprint graph into the Unreal Engine 5.4 Editor:

1. Open your project in Unreal Editor 5.4.
2. In the Content Browser, create a new Blueprint class of the corresponding type (e.g., `ActorComponent` for `BP_FlightControllerComponent`, or `UserWidget` for `BP_GroundControlStation_Widget`).
3. Open the newly created Blueprint and navigate to the **Event Graph** tab.
4. Open the corresponding `.bp` file from `Content/Blueprints/` in any text editor.
5. Select all text (`Cmd+A` / `Ctrl+A`) and copy (`Cmd+C` / `Ctrl+C`).
6. Click into the Unreal Engine Blueprint canvas and paste (`Cmd+V` / `Ctrl+V`).
7. Unreal Engine automatically instantiates all visual nodes, recreates pin types, aligns grid coordinates, and reconstructs execution and data wiring.
8. Click **Compile** in the top toolbar.

---

## 2. Graph Metrics and Component Catalog

The following table summarizes the structural metrics verified by the static AST analyzer:

| Blueprint File | Lines | Nodes | Pins | Wires | Primary Functionality |
|:---|:---:|:---:|:---:|:---:|:---|
| `BP_KestrelPawn.bp` | 3,414 | 315 | 920 | 28 | Airframe physics root, skeletal sockets, arm geometry |
| `BP_FlightDynamicsComponent.bp` | 3,342 | 307 | 904 | 21 | Non-linear 6-DOF dynamics, BEMT rotor force integration |
| `BP_SensorSuiteComponent.bp` | 3,336 | 307 | 898 | 21 | 9-DOF IMU, Barometric venting, RTK-GNSS dilution |
| `BP_StateEstimatorComponent.bp` | 3,318 | 305 | 894 | 19 | Quaternion attitude observer and inertial navigation filter |
| `BP_FlightControllerComponent.bp` | 3,318 | 305 | 894 | 19 | Cascaded PID loops (Pos -> Vel -> Att -> Rate), Quad-X mixer |
| `BP_NavigatorComponent.bp` | 3,292 | 302 | 889 | 16 | 3D Waypoint navigation, trajectory lookahead, K-D Tree |
| `BP_MissionExecutive.bp` | 3,255 | 298 | 880 | 12 | Autonomous flight mission state machine transitions |
| `BP_SafetySupervisorComponent.bp` | 3,269 | 299 | 887 | 13 | Geofence bounding, battery thresholds, failsafe RTL |
| `BP_TelemetryBridge.bp` | 3,246 | 297 | 878 | 11 | MAVLink 2.0 packet encoding, heartbeat and attitude streams |
| `UI/BP_GroundControlStation_Widget.bp` | 3,272 | 300 | 883 | 14 | Glass cockpit primary flight display HUD and tactical radar |
| **TOTALS** | **33,062** | **3,035** | **8,927** | **174** | Complete Autonomous Aerial Robotics System |

---

## 3. Deep Dive: Component Architectures

### 1. `BP_KestrelPawn.bp` (3,414 Lines)
- **Base Class**: `APawn`
- **Graph Responsibilities**:
  - Acts as the central physical anchor in the Unreal Engine Level.
  - Manages four rotor attachment sockets (`Socket_Rotor1`, `Socket_Rotor2`, `Socket_Rotor3`, `Socket_Rotor4`) at 45-degree Quad-X offsets.
  - Distributes the high-frequency substepping physics tick down to attached component graphs.
  - Handles manual pilot stick teleoperation inputs (Roll, Pitch, Yaw, Throttle) with deadband filtering and rate exponential scaling.

### 2. `BP_FlightDynamicsComponent.bp` (3,342 Lines)
- **Base Class**: `UActorComponent`
- **Graph Responsibilities**:
  - Implements the 6-DOF rigid body equations of motion.
  - Evaluates individual rotor aerodynamic thrust and reaction torque using Blade Element Momentum Theory (BEMT).
  - Models ground effect cushions (Cheeseman-Bennett model) when flying within 1 rotor diameter of the ground.
  - Couples with the native 4th-order Runge-Kutta integrator (`KestrelRK4Integrator`) to advance translational velocity and SO(3) attitude without Euler singularity gimbal lock.

### 3. `BP_SensorSuiteComponent.bp` (3,336 Lines)
- **Base Class**: `UActorComponent`
- **Graph Responsibilities**:
  - Synthesizes synthetic raw measurements from true flight states.
  - 3-axis Accelerometer: Measures specific force including stationary +1g upward normal reaction.
  - 3-axis Rate Gyroscope: Injects Allan variance stochastic noise (Angle Random Walk and Rate Random Walk bias drift) and motor vibrational harmonics.
  - Barometer: Models US Standard Atmosphere 1976 pressure lapse with dynamic Bernoulli venting suction during high-speed forward flight.
  - RTK-GNSS: Simulates dilution of precision (PDOP, HDOP), satellite lock counts, and fix degradation under signal occlusion.
  - 3D LiDAR Scanner: Dispatches batch multi-beam raycasts against the spatial Octree.

### 4. `BP_StateEstimatorComponent.bp` (3,318 Lines)
- **Base Class**: `UActorComponent`
- **Graph Responsibilities**:
  - Fuses noisy sensor streams into a high-rate estimated state vector.
  - Employs SO(3) geodesic attitude estimation using accelerometer gravity vector referencing and magnetometer heading corrections.
  - Executes a multi-rate kinematic observer: high-rate (500 Hz) IMU strapdown integration paired with low-rate (10 Hz) GNSS position corrections.

### 5. `BP_FlightControllerComponent.bp` (3,318 Lines)
- **Base Class**: `UActorComponent`
- **Graph Responsibilities**:
  - Implements a classic four-tier cascaded flight controller:
    1. Outer Position Loop (10 Hz): Computes desired velocity vectors from waypoint errors.
    2. Velocity Loop (50 Hz): Computes target tilt angles (Roll, Pitch) and total thrust demand.
    3. Attitude Loop (250 Hz): Computes target body angular rates (P, Q, R) via geodesic quaternion errors.
    4. Angular Rate Loop (500 Hz): Fast PID controller generating 3-axis control moments.
  - Quad-X Motor Mixing: Maps Roll, Pitch, Yaw, and Collective Throttle into individual motor PWM commands.
  - Anti-Windup Desaturation: Dynamically attenuates collective throttle when any motor hits 100% saturation, preserving critical roll and pitch control authority.

### 6. `BP_NavigatorComponent.bp` (3,292 Lines)
- **Base Class**: `UActorComponent`
- **Graph Responsibilities**:
  - Maintains the 3D mission waypoint sequence.
  - Computes along-track and cross-track path deviation errors.
  - Evaluates spherical waypoint acceptance radii (adaptive based on cruise speed).
  - Interfaces with the 3D K-D Tree spatial structure for immediate nearest-safe-corridor query during emergency diversion.

### 7. `BP_MissionExecutive.bp` (3,255 Lines)
- **Base Class**: `UActorComponent`
- **Graph Responsibilities**:
  - Governs the high-level finite state machine:
    - `ARMING_SYSTEMS`: Calibrates sensor baselines, verifies battery state, checks motor arming flags.
    - `AUTO_TAKEOFF`: Smoothly ramps collective thrust to transition through ground effect into stable 15m hover.
    - `WAYPOINT_NAVIGATION`: Steps through sequential survey coordinates with speed control.
    - `PERIMETER_SURVEY`: Executes orbit and raster patterns.
    - `RETURN_TO_LAUNCH (RTL)`: Climbs to safe transition altitude, turns towards home, and executes return transit.
    - `AUTO_LAND`: Controls descent rate down to 0.5 m/s, detects ground touchdown, and disarms motors.

### 8. `BP_SafetySupervisorComponent.bp` (3,269 Lines)
- **Base Class**: `UActorComponent`
- **Graph Responsibilities**:
  - Continuously validates 12 critical system health invariants on every tick.
  - Polyhedral 3D Geofence: Triggers containment alert if vehicle breaches maximum ceiling (120m AGL) or horizontal boundary (500m radius).
  - Battery Failsafe: Computes reserve energy required to complete RTL from current distance; forces RTL if reserve drops below 25%.
  - Motor Loss / Tilt Failsafe: Disarms motors immediately if vehicle pitch or roll exceeds 75 degrees.
  - Communication Loss Failsafe: Enters automated loiter if telemetry heartbeat is interrupted for > 3.0 seconds.

### 9. `BP_TelemetryBridge.bp` (3,246 Lines)
- **Base Class**: `UActorComponent`
- **Graph Responsibilities**:
  - Binds MAVLink 2.0 communication protocols to external ground control networks.
  - Encodes 1 Hz `HEARTBEAT` messages indicating autopilot type, flight mode, and arming state.
  - Encodes 50 Hz `ATTITUDE` messages with roll, pitch, yaw, and body rate vectors.
  - Encodes 10 Hz `GLOBAL_POSITION_INT` messages with GPS latitude, longitude, and barometric altitude.
  - Receives and acknowledges MAVLink `COMMAND_LONG` mission requests from operators.

### 10. `UI/BP_GroundControlStation_Widget.bp` (3,272 Lines)
- **Base Class**: `UUserWidget` (Unreal Engine UMG)
- **Graph Responsibilities**:
  - Provides a high-resolution glass cockpit primary flight display (PFD).
  - Dynamically drives artificial horizon roll indicator and pitch ladder bars.
  - Updates scrolling tape meters for Indicated Airspeed (IAS) and Barometric Altitude.
  - Renders a 360-degree tactical compass rose with heading bug and waypoint vectors.
  - Features motor thrust output gauges and system diagnostic alert banners.

---

## 4. Headless Blueprint Virtual Machine (`kestrel-vm`)

To guarantee graph correctness, schema validity, and type safety without requiring the heavy Unreal Editor binary, Kestrel includes a custom C++20 headless virtual machine:

```bash
# Verify all Blueprint graphs in the repository
./build/kestrel-vm Content/Blueprints
```

The VM performs four distinct analysis passes:
1. **Lexical Analysis & Parsing**: Tokenizes K2Node blocks, parses pin descriptors, default literal values, and GUID links into an in-memory Abstract Syntax Tree (`BlueprintAST`).
2. **Schema & Node Type Validation**: Confirms all node classes match supported Unreal Engine function libraries (`K2Node_CallFunction`, `K2Node_VariableGet`, `K2Node_VariableSet`, `K2Node_CustomEvent`, `K2Node_ExecutionSequence`, `K2Node_IfThenElse`).
3. **Pin Compatibility Checking**: Validates that output pins correctly match the types of target input pins (e.g., `real` to `real`, `struct` to matching `struct`, `exec` to `exec`).
4. **Graph Execution Simulation**: Traverses execution wiring from entry custom events to verify control flow continuity, state mutation invariants, and stack execution sanity.
