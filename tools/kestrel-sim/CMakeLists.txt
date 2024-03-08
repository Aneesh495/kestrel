cmake_minimum_required(VERSION 3.20)
project(kestrel_sim CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    src
    ../../Source/KestrelCore
)

add_definitions(-DKESTREL_STANDALONE=1)

set(SIM_SOURCES
    src/SITLSimulator.cpp
    ../../Source/KestrelCore/Math/KestrelQuaternionMath.cpp
    ../../Source/KestrelCore/Math/KestrelRK4Integrator.cpp
    ../../Source/KestrelCore/Aerodynamics/KestrelAtmosphere.cpp
    ../../Source/KestrelCore/Aerodynamics/KestrelWindModel.cpp
    ../../Source/KestrelCore/Aerodynamics/KestrelBladeElement.cpp
    ../../Source/KestrelCore/Sensors/KestrelNoiseModel.cpp
    ../../Source/KestrelCore/Sensors/KestrelSensorModels.cpp
    ../../Source/KestrelCore/Sensors/KestrelLiDARScanner.cpp
    ../../Source/KestrelCore/Spatial/KestrelOctree.cpp
    ../../Source/KestrelCore/Spatial/KestrelKDTree.cpp
    ../../Source/KestrelCore/Protocol/KestrelMAVLink.cpp
)

add_library(kestrel_sim_core STATIC ${SIM_SOURCES})

add_executable(kestrel-sim src/main.cpp)
target_link_libraries(kestrel-sim PRIVATE kestrel_sim_core)
