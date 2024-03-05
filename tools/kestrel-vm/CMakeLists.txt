cmake_minimum_required(VERSION 3.20)
project(kestrel_vm CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    include
    ../../Source/KestrelCore
)

add_definitions(-DKESTREL_STANDALONE=1)

set(VM_SOURCES
    src/BlueprintAST.cpp
    src/BlueprintParser.cpp
    src/BlueprintValidator.cpp
    src/BlueprintEvaluator.cpp
    ../../Source/KestrelCore/Math/KestrelQuaternionMath.cpp
)

add_library(kestrel_vm_core STATIC ${VM_SOURCES})

add_executable(kestrel-vm src/main.cpp)
target_link_libraries(kestrel-vm PRIVATE kestrel_vm_core)
