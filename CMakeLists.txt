cmake_minimum_required(VERSION 3.29)
project(com.arisen.physics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# Vendored solver (D18)
# ============================================================================

# Jolt.cmake builds the library from `Jolt/` and expects PHYSICS_REPO_ROOT to be the directory that
# contains it, which keeps the upstream include convention (`<Jolt/...>`) and source list intact.
set(PHYSICS_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/jolt")
include("${PHYSICS_REPO_ROOT}/Jolt/Jolt.cmake")

# ============================================================================
# Package payload
# ============================================================================

add_subdirectory(Physics)
