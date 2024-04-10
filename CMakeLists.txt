cmake_minimum_required(VERSION 3.20)
project(Kestrel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Standalone build definition for compiling KestrelCore outside Unreal Engine
add_definitions(-DKESTREL_STANDALONE=1)

# Root include paths
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Source/KestrelCore
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/kestrel-vm/include
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/kestrel-sim/src
)

# -----------------------------------------------------------------------------
# 1. Kestrel Core Primitives Library
# -----------------------------------------------------------------------------
set(KESTREL_CORE_SOURCES
    Source/KestrelCore/Math/KestrelQuaternionMath.cpp
    Source/KestrelCore/Math/KestrelRK4Integrator.cpp
    Source/KestrelCore/Aerodynamics/KestrelAtmosphere.cpp
    Source/KestrelCore/Aerodynamics/KestrelWindModel.cpp
    Source/KestrelCore/Aerodynamics/KestrelBladeElement.cpp
    Source/KestrelCore/Sensors/KestrelNoiseModel.cpp
    Source/KestrelCore/Sensors/KestrelSensorModels.cpp
    Source/KestrelCore/Sensors/KestrelLiDARScanner.cpp
    Source/KestrelCore/Spatial/KestrelOctree.cpp
    Source/KestrelCore/Spatial/KestrelKDTree.cpp
    Source/KestrelCore/Protocol/KestrelMAVLink.cpp
)

add_library(kestrel_core STATIC ${KESTREL_CORE_SOURCES})

# -----------------------------------------------------------------------------
# 2. Headless Blueprint Virtual Machine (kestrel-vm)
# -----------------------------------------------------------------------------
set(KESTREL_VM_SOURCES
    tools/kestrel-vm/src/BlueprintAST.cpp
    tools/kestrel-vm/src/BlueprintParser.cpp
    tools/kestrel-vm/src/BlueprintValidator.cpp
    tools/kestrel-vm/src/BlueprintEvaluator.cpp
)

add_library(kestrel_vm_lib STATIC ${KESTREL_VM_SOURCES})
target_link_libraries(kestrel_vm_lib PUBLIC kestrel_core)

add_executable(kestrel-vm tools/kestrel-vm/src/main.cpp)
target_link_libraries(kestrel-vm PRIVATE kestrel_vm_lib)

# -----------------------------------------------------------------------------
# 3. Software-In-The-Loop Flight Simulator (kestrel-sim)
# -----------------------------------------------------------------------------
set(KESTREL_SIM_SOURCES
    tools/kestrel-sim/src/SITLSimulator.cpp
)

add_library(kestrel_sim_lib STATIC ${KESTREL_SIM_SOURCES})
target_link_libraries(kestrel_sim_lib PUBLIC kestrel_core)

add_executable(kestrel-sim tools/kestrel-sim/src/main.cpp)
target_link_libraries(kestrel-sim PRIVATE kestrel_sim_lib)

# -----------------------------------------------------------------------------
# 4. Automated Test Suites & Regression Harness
# -----------------------------------------------------------------------------
enable_testing()

add_executable(test_quaternion_math tests/test_quaternion_math.cpp)
target_link_libraries(test_quaternion_math PRIVATE kestrel_core)
add_test(NAME QuaternionMath COMMAND test_quaternion_math)

add_executable(test_rk4_integrator tests/test_rk4_integrator.cpp)
target_link_libraries(test_rk4_integrator PRIVATE kestrel_core)
add_test(NAME RK4Integrator COMMAND test_rk4_integrator)

add_executable(test_spatial_structures tests/test_spatial_structures.cpp)
target_link_libraries(test_spatial_structures PRIVATE kestrel_core)
add_test(NAME SpatialStructures COMMAND test_spatial_structures)

add_executable(test_aerodynamics_bemt tests/test_aerodynamics_bemt.cpp)
target_link_libraries(test_aerodynamics_bemt PRIVATE kestrel_core)
add_test(NAME AerodynamicsBEMT COMMAND test_aerodynamics_bemt)

add_executable(test_sensor_models tests/test_sensor_models.cpp)
target_link_libraries(test_sensor_models PRIVATE kestrel_core)
add_test(NAME SensorModels COMMAND test_sensor_models)

add_executable(test_mavlink_crc tests/test_mavlink_crc.cpp)
target_link_libraries(test_mavlink_crc PRIVATE kestrel_core)
add_test(NAME MAVLinkCRC COMMAND test_mavlink_crc)

add_executable(test_blueprint_ast_verification tests/test_blueprint_ast_verification.cpp)
target_link_libraries(test_blueprint_ast_verification PRIVATE kestrel_vm_lib)
add_test(NAME BlueprintASTVerification COMMAND test_blueprint_ast_verification)

add_executable(test_sitl_integration tests/test_sitl_integration.cpp)
target_link_libraries(test_sitl_integration PRIVATE kestrel_sim_lib)
add_test(NAME SITLIntegration COMMAND test_sitl_integration)

add_executable(test_wind_gust_rejection tests/test_wind_gust_rejection.cpp)
target_link_libraries(test_wind_gust_rejection PRIVATE kestrel_sim_lib)
add_test(NAME WindGustRejection COMMAND test_wind_gust_rejection)

add_executable(test_regression_bugs tests/test_regression_bugs.cpp)
target_link_libraries(test_regression_bugs PRIVATE kestrel_core)
add_test(NAME RegressionBugs COMMAND test_regression_bugs)
