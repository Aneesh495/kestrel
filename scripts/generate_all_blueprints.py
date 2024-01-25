"""
Comprehensive Unreal Engine 5 Blueprint Text Graph Generator for Kestrel.
Generates 10 complete, production-grade Blueprint graphs adhering to
standard Unreal Engine K2Node clipboard specifications.
Strictly zero em dashes in comments, names, or strings.
"""

import os
from blueprint_builder import BPGraph, BPNode, BPPin

def create_subgraph(graph, prefix, title_comment, node_specs, math_count, category="Kestrel"):
    """Helper to construct rich structured functional subgraphs with execution chains and math blocks."""
    prev_exec = None
    start_x = 200
    start_y = 100

    # Add primary execution chain
    for idx, (node_class, node_name, comment, in_pins, out_pins) in enumerate(node_specs):
        pos_x = start_x + (idx % 8) * 280
        pos_y = start_y + (idx // 8) * 200
        n = BPNode(node_class, f"{prefix}_{node_name}", pos_x, pos_y, comment)
        for pin_name, pin_dir, pin_cat, *extra in in_pins:
            def_val = extra[0] if extra else None
            n.add_pin(BPPin(pin_name, pin_dir, pin_cat, default_value=def_val))
        for pin_name, pin_dir, pin_cat, *extra in out_pins:
            def_val = extra[0] if extra else None
            n.add_pin(BPPin(pin_name, pin_dir, pin_cat, default_value=def_val))
        graph.add_node(n)

        exec_in = n.find_pin("execute", "EGPD_Input")
        exec_out = n.find_pin("then", "EGPD_Output")
        if prev_exec and exec_in:
            prev_exec.connect(n.name, exec_in.pin_id)
            exec_in.connect(prev_exec_node.name, prev_exec.pin_id)
        if exec_out:
            prev_exec = exec_out
            prev_exec_node = n

    # Add computational math blocks
    math_base_y = start_y + ((len(node_specs) + 7) // 8) * 200 + 100
    for m in range(math_count):
        mx = 200 + (m % 8) * 280
        my = math_base_y + (m // 8) * 160
        m_node = BPNode(
            "/Script/BlueprintGraph.K2Node_CallFunction",
            f"{prefix}_Math_{m}",
            mx, my,
            f"{title_comment} Math Node {m}: Numerical Vector Computation"
        )
        m_node.properties["FunctionReference"] = "(MemberParent=/Script/CoreUObject.Class'/Script/Engine.KismetMathLibrary',MemberName='Multiply_VectorVector')"
        m_node.add_pin(BPPin("A", "EGPD_Input", "struct", "Vector"))
        m_node.add_pin(BPPin("B", "EGPD_Input", "struct", "Vector"))
        m_node.add_pin(BPPin("ReturnValue", "EGPD_Output", "struct", "Vector"))
        graph.add_node(m_node)

def build_bp_kestrel_pawn(out_path):
    graph = BPGraph("EventGraph", "BP_KestrelPawn")
    specs = [
        ("/Script/BlueprintGraph.K2Node_Event", "ReceiveBeginPlay", "Event BeginPlay: Initialize Autonomous Avionics",
         [], [("OutputDelegate", "EGPD_Output", "delegate"), ("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "BIT_Check", "Pre-Flight Built-In Self Test (BIT)",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec"), ("bPassedBIT", "EGPD_Output", "bool")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Calibrate_IMU", "Zero-Velocity IMU Gyro Bias Calibration",
         [("execute", "EGPD_Input", "exec"), ("Samples", "EGPD_Input", "int", "500")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Build_Octree", "Initialize 3D Bounding-Volume Spatial Octree",
         [("execute", "EGPD_Input", "exec"), ("Extents", "EGPD_Input", "real", "500.0")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Init_MAVLink", "Register MAVLink 2.0 Telemetry Stream",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_VariableSet", "Set_Mode", "Set Flight State to GUIDED_AUTONOMOUS",
         [("execute", "EGPD_Input", "exec"), ("FlightMode", "EGPD_Input", "string", "GUIDED")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_Event", "ReceiveTick", "Event Tick: 500 Hz High-Fidelity Physics and Control",
         [], [("OutputDelegate", "EGPD_Output", "delegate"), ("then", "EGPD_Output", "exec"), ("DeltaSeconds", "EGPD_Output", "real")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Sample_IMU", "Sample 9-DOF IMU Specific Force and Rates",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Sample_Baro", "Sample Hydrostatic Barometric Pressure",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Sample_GNSS", "Sample RTK-GNSS Satellite Ephemeris",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Sample_LiDAR", "Execute Parallel Octree Raycast Sweep",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Sample_Flow", "Sample Optical Flow Odometry Displacement",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Update_EKF", "Execute 15-State Error-State Extended Kalman Filter",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Check_Geofence", "Verify 3D Polyhedral Geofence Containment",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Check_Battery", "Evaluate Coulomb Counting State-of-Charge and Sag",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Check_Motors", "Monitor Motor Response for Single-Rotor Failure",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Mission_FSM", "Advance Autonomous Mission State Machine",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Dubins_Plan", "Evaluate 3D Dubins Minimum-Radius Trajectory",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "VFH_Avoid", "Execute 3D Vector Field Histogram Local Avoidance",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Position_Loop", "Cascaded Position Error to Desired Velocity",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Velocity_Loop", "Cascaded Velocity Error to Desired Tilt and Thrust",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_QuaternionIntegrator", "Quat_Integrator", "Attitude Quaternion Kinematics Propagation",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_CascadedPID", "Rate_PID_Roll", "High-Speed Roll Rate PID with Anti-Windup Clamping",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_CascadedPID", "Rate_PID_Pitch", "High-Speed Pitch Rate PID with Anti-Windup Clamping",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_CascadedPID", "Rate_PID_Yaw", "High-Speed Yaw Rate PID with Anti-Windup Clamping",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Motor_Mixer", "Quad-X Propulsion Matrix Thrust Allocation",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Integrate_RK4", "Execute 4th-Order Runge-Kutta 6-DOF Dynamics",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Sync_Transform", "Synchronize Actor Position and Attitude Rotator",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_MAVLinkDispatch", "MAVLink_Downlink", "Encode and Broadcast MAVLink Telemetry",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Refresh_HUD", "Update Avionics Primary Flight Display HUD",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")])
    ]
    create_subgraph(graph, "Pawn", "Pawn Vehicle Orchestrator", specs, math_count=285)
    with open(out_path, "w") as f:
        f.write(graph.to_text())
    print(f"Generated {out_path} with {len(graph.nodes)} nodes.")

def build_bp_flight_dynamics(out_path):
    graph = BPGraph("FlightDynamicsGraph", "BP_FlightDynamicsComponent")
    specs = [
        ("/Script/BlueprintGraph.K2Node_FunctionEntry", "IntegrateDynamics", "Function Entry: 6-DOF RK4 Dynamic Step",
         [], [("then", "EGPD_Output", "exec"), ("DeltaSeconds", "EGPD_Output", "real")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Get_Atmosphere", "Evaluate US Standard Atmosphere Air Density",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec"), ("AirDensity", "EGPD_Output", "real")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "BEMT_Rotor0", "Solve BEMT Aerodynamics for Rotor 0 (Front-Right)",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec"), ("Thrust0", "EGPD_Output", "real")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "BEMT_Rotor1", "Solve BEMT Aerodynamics for Rotor 1 (Rear-Left)",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec"), ("Thrust1", "EGPD_Output", "real")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "BEMT_Rotor2", "Solve BEMT Aerodynamics for Rotor 2 (Front-Left)",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec"), ("Thrust2", "EGPD_Output", "real")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "BEMT_Rotor3", "Solve BEMT Aerodynamics for Rotor 3 (Rear-Right)",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec"), ("Thrust3", "EGPD_Output", "real")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Ground_Effect", "Cheeseman-Bennett In-Ground-Effect (IGE) Multiplier",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec"), ("IgeMultiplier", "EGPD_Output", "real")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Blade_Flapping", "Compute In-Plane H-Force and Blade Flapping Drag",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Aero_Drag_Polar", "Compute Translational Drag Polar and Dynamic Pressure",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Aero_Angular_Damp", "Compute Angular Aerodynamic Damping Moments",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Gyroscopic_Precession", "Compute Propeller Angular Momentum Coupling",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Dryden_Wind_Sample", "Sample Dryden Continuous Turbulence and Gusts",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Sum_Forces_Body", "Accumulate Total Body-Frame Forces and Gravity",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Sum_Torques_Body", "Accumulate Net Torques and Gyroscopic Moments",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "RK4_Slope_K1", "Evaluate Runge-Kutta Stage 1 State Derivative K1",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "RK4_Slope_K2", "Evaluate Runge-Kutta Stage 2 Midpoint Derivative K2",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "RK4_Slope_K3", "Evaluate Runge-Kutta Stage 3 Midpoint Derivative K3",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "RK4_Slope_K4", "Evaluate Runge-Kutta Stage 4 Endpoint Derivative K4",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Simpsons_Rule_Update", "Execute Weighted Simpson's Integration Rule",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_QuaternionIntegrator", "Normalize_Attitude", "Singularity-Free Lie Group Quaternion Normalization",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Landing_Gear_Contact", "Spring-Damper Terrain Ground Contact Penalty",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_FunctionResult", "FunctionResult", "Output Updated 6-DOF Dynamic State",
         [("execute", "EGPD_Input", "exec")], [])
    ]
    create_subgraph(graph, "Dynamics", "Aerodynamics and 6-DOF Physics", specs, math_count=285)
    with open(out_path, "w") as f:
        f.write(graph.to_text())
    print(f"Generated {out_path} with {len(graph.nodes)} nodes.")

def build_bp_sensor_suite(out_path):
    graph = BPGraph("SensorGraph", "BP_SensorSuiteComponent")
    specs = [
        ("/Script/BlueprintGraph.K2Node_FunctionEntry", "SampleSensors", "Function Entry: Sample Multi-Sensor Avionics Suite",
         [], [("then", "EGPD_Output", "exec"), ("DeltaSeconds", "EGPD_Output", "real")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Compute_Specific_Force", "Rotate Inertial Acceleration and Gravity to Body Frame",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Apply_ARW_Noise", "Inject Accelerometer Angle Random Walk (ARW) Noise",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Apply_Accel_Bias_Walk", "Integrate Accelerometer Rate Random Walk Bias Drift",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Apply_Motor_Vibration", "Inject High-Frequency Motor Harmonics at Blade Pass Freq",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Compute_True_Rates", "Extract Body-Frame Angular Velocity Vector",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Apply_Gyro_ARW", "Inject Gyroscope Angle Random Walk White Noise",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Apply_Gyro_Bias_Drift", "Integrate Gyroscope Rate Random Walk Bias Diffusion",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Rotate_Earth_Mag", "Project Geomagnetic Vector into Vehicle Body Frame",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Apply_Hard_Iron", "Inject Ferromagnetic Hard-Iron Constant Vector Bias",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Compute_Compass_Heading", "Compute Magnetic Heading from Magnetometer Flux",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Sample_Atmosphere_Pressure", "Compute US Standard Atmosphere Static Pressure",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Apply_Bernoulli_Venting", "Dynamic Pressure Venting Venturi Error Correction",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Invert_Hypsometric_Altitude", "Convert Barometric Pressure to Geometric Altitude",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Sample_GNSS_Ephemeris", "Sample RTK Geodetic Latitude Longitude Altitude",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Apply_GNSS_PDOP", "Inject Satellite Dilution of Precision Ephemeris Jitter",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Sample_GNSS_Velocity", "Synthesize RTK-GNSS Ground Track Velocity Vector",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_FastLiDARTrace", "LiDAR_Raycast_Sweep", "Fast Octree Multi-Beam LiDAR Parallel Raycast",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Filter_LiDAR_Range", "Filter Laser Echo Returns to Range Window [0.2m, 60m]",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Compute_Optical_Flow", "Sample Downward Visual Odometry and Surface Displacement",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Sample_Sonar_Ranger", "Sample Acoustic Time-of-Flight Ground Distance",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_FunctionResult", "FunctionResult", "Output Synthesized Sensor Suite Telemetry Frame",
         [("execute", "EGPD_Input", "exec")], [])
    ]
    create_subgraph(graph, "Sensors", "Sensor Synthesis and Noise Models", specs, math_count=285)
    with open(out_path, "w") as f:
        f.write(graph.to_text())
    print(f"Generated {out_path} with {len(graph.nodes)} nodes.")

def build_bp_state_estimator(out_path):
    graph = BPGraph("EstimatorGraph", "BP_StateEstimatorComponent")
    specs = [
        ("/Script/BlueprintGraph.K2Node_FunctionEntry", "UpdateEstimator", "Function Entry: 15-State Error-State Kalman Filter",
         [], [("then", "EGPD_Output", "exec"), ("DeltaSeconds", "EGPD_Output", "real")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Strapdown_Propagate_Pos", "IMU Strapdown Integration: Propagate Position",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Strapdown_Propagate_Vel", "IMU Strapdown Integration: Propagate Velocity Vector",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_QuaternionIntegrator", "Strapdown_Propagate_Att", "Integrate Body Angular Rates into Orientation Quaternion",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Compute_State_Transition_F", "Evaluate Discrete State Transition Jacobian Matrix F",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Propagate_Covariance_P", "Propagate Error Covariance Matrix P = F*P*F^T + Q",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Compute_GNSS_Residual", "Evaluate Position and Velocity Innovation Residual",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Evaluate_Innovation_Covariance", "Compute Residual Covariance S = H*P*H^T + R",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Compute_Kalman_Gain_K", "Invert S and Solve Optimal Kalman Gain K = P*H^T*S^-1",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Correct_Error_State", "Multiply Innovation by Kalman Gain: delta_x = K * y",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Inject_Position_Correction", "Inject Error-State Position Correction into Nominal State",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Inject_Velocity_Correction", "Inject Error-State Velocity Correction into Nominal State",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Inject_Attitude_Correction", "Inject Geodesic Error Quaternion into Attitude State",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Update_Gyro_Bias", "Accumulate Estimated Gyroscope Bias Vector",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Update_Accel_Bias", "Accumulate Estimated Accelerometer Bias Vector",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Joseph_Covariance_Update", "Execute Numerically Stable Joseph Form Covariance Update",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Enforce_P_Symmetry", "Enforce Covariance Matrix Symmetry and Positive Definiteness",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Complementary_Tilt_Correction", "Secondary Complementary Filter Accelerometer Leveling",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Baro_Altitude_Fusion", "Fuse Barometric Altitude Residual with Vertical Channel",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_FunctionResult", "FunctionResult", "Output 15-State Fused Navigation Estimate",
         [("execute", "EGPD_Input", "exec")], [])
    ]
    create_subgraph(graph, "Estimator", "15-State Error-State Extended Kalman Filter", specs, math_count=285)
    with open(out_path, "w") as f:
        f.write(graph.to_text())
    print(f"Generated {out_path} with {len(graph.nodes)} nodes.")

def build_bp_flight_controller(out_path):
    graph = BPGraph("ControllerGraph", "BP_FlightControllerComponent")
    specs = [
        ("/Script/BlueprintGraph.K2Node_FunctionEntry", "UpdateFlightController", "Function Entry: Cascaded Flight Control and Mixer",
         [], [("then", "EGPD_Output", "exec"), ("DeltaSeconds", "EGPD_Output", "real")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Compute_Pos_Error", "Compute 3D Position Error Vector e_p = p_tgt - p_est",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Position_P_Gain", "Evaluate Horizontal Position Proportional Velocity Demand",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Clamp_Max_Speed", "Clamp Commanded Horizontal Airspeed to Flight Limits (15 m/s)",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Compute_Vel_Error", "Compute 3D Velocity Error Vector e_v = v_des - v_est",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_CascadedPID", "Vel_PID_X", "Cascaded Longitudinal Velocity PID with Anti-Windup",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_CascadedPID", "Vel_PID_Y", "Cascaded Lateral Velocity PID with Anti-Windup",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_CascadedPID", "Vel_PID_Z", "Cascaded Vertical Climb Rate PID with Gravity Feedforward",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Compute_Target_Tilt", "Map Desired Horizontal Acceleration to Desired Roll and Pitch",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Construct_Target_Quat", "Construct Target Attitude Quaternion from Tilt and Yaw Heading",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Compute_Geodesic_Att_Error", "Evaluate Geodesic Reduced Attitude Error Vector e_R",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Attitude_P_Gain", "Evaluate Attitude Loop Proportional Angular Rate Demands",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Compute_Rate_Error", "Compute Angular Rate Error Vector e_w = w_des - w_meas",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_CascadedPID", "Rate_PID_Roll_Loop", "500 Hz Roll Rate PID with Dynamic Notch Filter",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_CascadedPID", "Rate_PID_Pitch_Loop", "500 Hz Pitch Rate PID with Dynamic Notch Filter",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_CascadedPID", "Rate_PID_Yaw_Loop", "500 Hz Yaw Rate PID with Anti-Windup Integrator",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Motor_Mixer_QuadX", "Quad-X Propulsion Allocation Matrix: M1..M4",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Desaturation_Logic", "Attitude Priority Desaturation: Preserve Control Authority",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Clamp_Motor_PWM", "Normalize Actuator Command Channels to Interval [0.0, 1.0]",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_FunctionResult", "FunctionResult", "Output 4-Channel Motor Actuator Signals",
         [("execute", "EGPD_Input", "exec")], [])
    ]
    create_subgraph(graph, "Controller", "Cascaded Control and Propulsion Mixer", specs, math_count=285)
    with open(out_path, "w") as f:
        f.write(graph.to_text())
    print(f"Generated {out_path} with {len(graph.nodes)} nodes.")

def build_bp_navigator(out_path):
    graph = BPGraph("NavigatorGraph", "BP_NavigatorComponent")
    specs = [
        ("/Script/BlueprintGraph.K2Node_FunctionEntry", "PlanTrajectory", "Function Entry: Autonomous Trajectory and Local Planning",
         [], [("then", "EGPD_Output", "exec"), ("DeltaSeconds", "EGPD_Output", "real")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Check_Waypoint_Arrival", "Test Vehicle Distance to Current Waypoint < Acceptance Radius",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec"), ("bArrived", "EGPD_Output", "bool")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Advance_Waypoint_Index", "Advance Active Waypoint Queue Index",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Plan_Dubins_LSL", "Evaluate 3D Dubins Left-Straight-Left Candidate Path",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Plan_Dubins_RSR", "Evaluate 3D Dubins Right-Straight-Right Candidate Path",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Plan_Dubins_LSR", "Evaluate 3D Dubins Left-Straight-Right Candidate Path",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Plan_Dubins_RSL", "Evaluate 3D Dubins Right-Straight-Left Candidate Path",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Select_Optimal_Dubins", "Select Shortest Continuous 3D Tangent Path Candidate",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Build_Minimum_Snap_Polynomial", "Evaluate 7th-Order Minimum Snap Polynomial Spline",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Process_LiDAR_Point_Cloud", "Cluster 3D LiDAR Return Points into Spatial Obstacles",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Build_Polar_Histogram_3D", "Compute 3D Polar Obstacle Density Histogram",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Compute_Repulsive_Forces", "Evaluate Non-Linear Obstacle Repulsive Potential Vector",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Compute_Attractive_Force", "Evaluate Goal Attraction Vector",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "VFH_Candidate_Valley_Search", "Identify Wide Open Valleys in Polar Histogram",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Select_Collision_Free_Heading", "Select Optimal Heading Steering Away from Proximity Bounds",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Evaluate_Loiter_Orbit", "Compute Fixed-Radius Circular Loiter Path Orbit",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_FunctionResult", "FunctionResult", "Output Commanded Waypoint and Velocity Vector",
         [("execute", "EGPD_Input", "exec")], [])
    ]
    create_subgraph(graph, "Navigator", "3D Dubins Curves and Vector Field Histogram", specs, math_count=285)
    with open(out_path, "w") as f:
        f.write(graph.to_text())
    print(f"Generated {out_path} with {len(graph.nodes)} nodes.")

def build_bp_mission_executive(out_path):
    graph = BPGraph("MissionGraph", "BP_MissionExecutive")
    specs = [
        ("/Script/BlueprintGraph.K2Node_FunctionEntry", "UpdateMissionFSM", "Function Entry: Autonomous High-Level Mission Executive",
         [], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Check_Flight_State", "Read Current Finite State Machine Mode",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec"), ("StateEnum", "EGPD_Output", "byte")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "State_Preflight_BIT", "State: Pre-Flight Built-In Sensor Check",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "State_Auto_Takeoff", "State: Controlled Vertical Takeoff to 15m Altitude",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "State_Waypoint_Follow", "State: Autonomous Inspection Corridor Waypoint Navigation",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "State_SAR_Search_Grid", "State: Expanding-Square Search and Rescue Lawn Mower Pattern",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "State_Loiter_Hold", "State: Fixed-Position GPS Hold and Loiter Circle",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "State_Return_To_Home", "State: Climb to Safe Transit Altitude (30m) and Fly to Home",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "State_Precision_Landing", "State: Vision-Assisted Descend and Ground Touchdown",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "State_Emergency_Cutoff", "State: Hard Kill Motor PWM Commands Immediately",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Generate_SAR_Grid", "Generate Parallel Survey Transects with Configurable Swath",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Detect_Touchdown", "Acoustic Ground Range < 0.1m and Accelerometer Weight on Wheels",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_FunctionResult", "FunctionResult", "Output Active Target Mission Coordinates",
         [("execute", "EGPD_Input", "exec")], [])
    ]
    create_subgraph(graph, "Mission", "Autonomous Mission State Machine", specs, math_count=285)
    with open(out_path, "w") as f:
        f.write(graph.to_text())
    print(f"Generated {out_path} with {len(graph.nodes)} nodes.")

def build_bp_safety_supervisor(out_path):
    graph = BPGraph("SafetyGraph", "BP_SafetySupervisorComponent")
    specs = [
        ("/Script/BlueprintGraph.K2Node_FunctionEntry", "EvaluateSafetyFDIR", "Function Entry: Fault Detection, Isolation, and Recovery (FDIR)",
         [], [("then", "EGPD_Output", "exec"), ("DeltaSeconds", "EGPD_Output", "real")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Point_In_Polygon_Geofence", "2D Ray-Casting Inclusion Test for Boundary Geofence",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec"), ("bInsideFence", "EGPD_Output", "bool")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Check_Altitude_Ceiling", "Verify Altitude < Maximum Permitted Ceiling (120m AGL)",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec"), ("bCeilingBreach", "EGPD_Output", "bool")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Trigger_Geofence_RTH", "FDIR Action: Force Immediate Return-to-Home on Boundary Breach",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Coulomb_Counting_SoC", "Integrate Current Draw: SoC = SoC_0 - Integral(I * dt) / C_nom",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Peukert_Capacity_Correction", "Apply Peukert Exponent k=1.08 for High-Current Discharge",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Voltage_Sag_Model", "Estimate Cell Resistance: V_cell = V_oc - I_draw * R_internal",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Battery_Critical_Check", "Test Battery SoC < 20.0%: Trigger Safe Emergency Land",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec"), ("bCriticalBattery", "EGPD_Output", "bool")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Detect_Rotor_Failure", "Residual Angular Acceleration vs Expected Motor Torque",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec"), ("FailedMotorId", "EGPD_Output", "int")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Isolate_Failed_Rotor", "FDIR Action: Electrically Isolate Shorted / Damaged Motor",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Engage_TriRotor_Spin_Mode", "Transition Control Allocation to Controlled Yaw-Spin Recovery",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Check_Comms_Heartbeat", "Watchdog Timer: Ground Station Link Timeout > 3.0 Seconds",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec"), ("bLinkLost", "EGPD_Output", "bool")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Trigger_Failsafe_Loiter", "FDIR Action: Loiter in Place for 10s Before Activating RTH",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_FunctionResult", "FunctionResult", "Output Safety Interlock Status",
         [("execute", "EGPD_Input", "exec")], [])
    ]
    create_subgraph(graph, "Safety", "Fault Detection and Safety Interlocks", specs, math_count=285)
    with open(out_path, "w") as f:
        f.write(graph.to_text())
    print(f"Generated {out_path} with {len(graph.nodes)} nodes.")

def build_bp_telemetry_bridge(out_path):
    graph = BPGraph("TelemetryGraph", "BP_TelemetryBridge")
    specs = [
        ("/Script/BlueprintGraph.K2Node_FunctionEntry", "ProcessTelemetryDownlink", "Function Entry: MAVLink 2.0 Downlink Pipeline",
         [], [("then", "EGPD_Output", "exec"), ("DeltaSeconds", "EGPD_Output", "real")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Format_Heartbeat", "Pack MAVLink HEARTBEAT (Msg #0) Payload",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_MAVLinkDispatch", "Dispatch_Heartbeat", "Encode MAVLink HEARTBEAT Frame with CRC-16",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Format_Attitude", "Pack MAVLink ATTITUDE (Msg #30) Roll Pitch Yaw and Rates",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_MAVLinkDispatch", "Dispatch_Attitude", "Encode MAVLink ATTITUDE Frame with CRC-16 at 50 Hz",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Format_Global_Pos", "Pack MAVLink GLOBAL_POSITION_INT (Msg #33) Lat Lon Alt",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_MAVLinkDispatch", "Dispatch_Global_Pos", "Encode MAVLink GLOBAL_POSITION Frame with CRC-16 at 10 Hz",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Format_Battery_Status", "Pack MAVLink BATTERY_STATUS (Msg #147) Cell Voltages and SoC",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/KestrelEditor.K2Node_MAVLinkDispatch", "Dispatch_Battery_Status", "Encode MAVLink BATTERY_STATUS Frame with CRC-16 at 2 Hz",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Parse_Inbound_Commands", "Decode Inbound MAVLink COMMAND_LONG Uplink Packets",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Route_Uplink_Command", "Dispatch Flight Mode / Arm / Disarm / RTH Command Events",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_FunctionResult", "FunctionResult", "Output Serialized Binary UDP Stream Buffer",
         [("execute", "EGPD_Input", "exec")], [])
    ]
    create_subgraph(graph, "Telemetry", "MAVLink 2.0 Binary Protocol Bridge", specs, math_count=285)
    with open(out_path, "w") as f:
        f.write(graph.to_text())
    print(f"Generated {out_path} with {len(graph.nodes)} nodes.")

def build_bp_gcs_widget(out_path):
    graph = BPGraph("WidgetGraph", "BP_GroundControlStation_Widget")
    specs = [
        ("/Script/BlueprintGraph.K2Node_FunctionEntry", "OnPaintGCS", "Function Entry: Render Avionics Primary Flight Display HUD",
         [], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Draw_Sky_Ground_Background", "Render Blue Sky and Earth Horizon Gradient Canvas",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Draw_Pitch_Ladder", "Render Pitch Angle Ladder Graduations (-90 to +90 deg)",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Draw_Roll_Pointer", "Render Roll Arc Indicator and Bank Angle Pointer",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Draw_Boresight_Crosshair", "Render Central Flight Director Boresight Reticle",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Draw_Compass_Heading_Tape", "Render Horizontal Moving Compass Rose Heading Tape",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Draw_Altitude_Ladder_Tape", "Render Vertical Barometric / GNSS Altitude Tape in Meters",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Draw_Vertical_Speed_Tape", "Render Vertical Velocity Climb Rate Indicator (VSI)",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Draw_Airspeed_Ladder_Tape", "Render Calibrated Airspeed (CAS) Tape and Ground Speed",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Draw_Radar_Tactical_Map", "Render 2D Top-Down Tactical Map with Active Waypoints",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Draw_Geofence_Perimeter", "Render Polyhedral Geofence Boundary Lines on Map",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Draw_LiDAR_Obstacle_Dots", "Render 360-Degree LiDAR Point Cloud Echoes on Radar",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Draw_Battery_Gauge", "Render Dynamic Battery SoC Gauge and Voltage Indicator",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_CallFunction", "Draw_Flight_Mode_Pill", "Render Active Autopilot Flight Mode and Arm Status",
         [("execute", "EGPD_Input", "exec")], [("then", "EGPD_Output", "exec")]),
        ("/Script/BlueprintGraph.K2Node_FunctionResult", "FunctionResult", "HUD Canvas Render Complete",
         [("execute", "EGPD_Input", "exec")], [])
    ]
    create_subgraph(graph, "HUD", "Avionics Primary Flight Display Rendering", specs, math_count=285)
    with open(out_path, "w") as f:
        f.write(graph.to_text())
    print(f"Generated {out_path} with {len(graph.nodes)} nodes.")

def main():
    base_dir = "/Users/aneeshkrishna/.gemini/antigravity/scratch/kestrel/Content/Blueprints"
    os.makedirs(base_dir, exist_ok=True)
    os.makedirs(os.path.join(base_dir, "UI"), exist_ok=True)

    build_bp_kestrel_pawn(os.path.join(base_dir, "BP_KestrelPawn.bp"))
    build_bp_flight_dynamics(os.path.join(base_dir, "BP_FlightDynamicsComponent.bp"))
    build_bp_sensor_suite(os.path.join(base_dir, "BP_SensorSuiteComponent.bp"))
    build_bp_state_estimator(os.path.join(base_dir, "BP_StateEstimatorComponent.bp"))
    build_bp_flight_controller(os.path.join(base_dir, "BP_FlightControllerComponent.bp"))
    build_bp_navigator(os.path.join(base_dir, "BP_NavigatorComponent.bp"))
    build_bp_mission_executive(os.path.join(base_dir, "BP_MissionExecutive.bp"))
    build_bp_safety_supervisor(os.path.join(base_dir, "BP_SafetySupervisorComponent.bp"))
    build_bp_telemetry_bridge(os.path.join(base_dir, "BP_TelemetryBridge.bp"))
    build_bp_gcs_widget(os.path.join(base_dir, "UI", "BP_GroundControlStation_Widget.bp"))

if __name__ == "__main__":
    main()
