cmake_minimum_required(VERSION 3.20)
project(kestrel_tests CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../Source/KestrelCore
    ${CMAKE_CURRENT_SOURCE_DIR}/../tools/kestrel-vm/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../tools/kestrel-sim/src
)

add_definitions(-DKESTREL_STANDALONE=1)

# Core library for test dependencies
set(TEST_CORE_SOURCES
    ../Source/KestrelCore/Math/KestrelQuaternionMath.cpp
    ../Source/KestrelCore/Math/KestrelRK4Integrator.cpp
    ../Source/KestrelCore/Aerodynamics/KestrelAtmosphere.cpp
    ../Source/KestrelCore/Aerodynamics/KestrelWindModel.cpp
    ../Source/KestrelCore/Aerodynamics/KestrelBladeElement.cpp
    ../Source/KestrelCore/Sensors/KestrelNoiseModel.cpp
    ../Source/KestrelCore/Sensors/KestrelSensorModels.cpp
    ../Source/KestrelCore/Sensors/KestrelLiDARScanner.cpp
    ../Source/KestrelCore/Spatial/KestrelOctree.cpp
    ../Source/KestrelCore/Spatial/KestrelKDTree.cpp
    ../Source/KestrelCore/Protocol/KestrelMAVLink.cpp
    ../tools/kestrel-sim/src/SITLSimulator.cpp
    ../tools/kestrel-vm/src/BlueprintAST.cpp
    ../tools/kestrel-vm/src/BlueprintParser.cpp
    ../tools/kestrel-vm/src/BlueprintValidator.cpp
    ../tools/kestrel-vm/src/BlueprintEvaluator.cpp
)

add_library(kestrel_test_framework STATIC ${TEST_CORE_SOURCES})

enable_testing()

# 1. Quaternion Math & SO(3)
add_executable(test_quaternion_math test_quaternion_math.cpp)
target_link_libraries(test_quaternion_math PRIVATE kestrel_test_framework)
add_test(NAME QuaternionMath COMMAND test_quaternion_math)

# 2. Runge-Kutta 4th Order Integrator
add_executable(test_rk4_integrator test_rk4_integrator.cpp)
target_link_libraries(test_rk4_integrator PRIVATE kestrel_test_framework)
add_test(NAME RK4Integrator COMMAND test_rk4_integrator)

# 3. Spatial Structures (Octree, K-D Tree)
add_executable(test_spatial_structures test_spatial_structures.cpp)
target_link_libraries(test_spatial_structures PRIVATE kestrel_test_framework)
add_test(NAME SpatialStructures COMMAND test_spatial_structures)

# 4. Aerodynamics & BEMT
add_executable(test_aerodynamics_bemt test_aerodynamics_bemt.cpp)
target_link_libraries(test_aerodynamics_bemt PRIVATE kestrel_test_framework)
add_test(NAME AerodynamicsBEMT COMMAND test_aerodynamics_bemt)

# 5. Sensor Models & Noise
add_executable(test_sensor_models test_sensor_models.cpp)
target_link_libraries(test_sensor_models PRIVATE kestrel_test_framework)
add_test(NAME SensorModels COMMAND test_sensor_models)

# 6. MAVLink Wire Protocol & CRC-16
add_executable(test_mavlink_crc test_mavlink_crc.cpp)
target_link_libraries(test_mavlink_crc PRIVATE kestrel_test_framework)
add_test(NAME MAVLinkCRC COMMAND test_mavlink_crc)

# 7. Blueprint AST Static Verification
add_executable(test_blueprint_ast_verification test_blueprint_ast_verification.cpp)
target_link_libraries(test_blueprint_ast_verification PRIVATE kestrel_test_framework)
add_test(NAME BlueprintASTVerification COMMAND test_blueprint_ast_verification)

# 8. SITL End-to-End Integration Mission
add_executable(test_sitl_integration test_sitl_integration.cpp)
target_link_libraries(test_sitl_integration PRIVATE kestrel_test_framework)
add_test(NAME SITLIntegration COMMAND test_sitl_integration)

# 9. Wind Gust Disturbance Rejection
add_executable(test_wind_gust_rejection test_wind_gust_rejection.cpp)
target_link_libraries(test_wind_gust_rejection PRIVATE kestrel_test_framework)
add_test(NAME WindGustRejection COMMAND test_wind_gust_rejection)

# 10. Documented Regression Bug Suite
add_executable(test_regression_bugs test_regression_bugs.cpp)
target_link_libraries(test_regression_bugs PRIVATE kestrel_test_framework)
add_test(NAME RegressionBugs COMMAND test_regression_bugs)
