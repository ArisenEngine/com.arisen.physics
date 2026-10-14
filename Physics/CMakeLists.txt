# The package payload: one shared library with a flat C ABI that the managed service binds to. Solver
# types stay behind this boundary, so no gameplay or engine package ever includes a Jolt header.
add_library(Physics SHARED
    PhysicsBridge.cpp
    PhysicsBridge.h
)

target_include_directories(Physics PRIVATE "${PHYSICS_REPO_ROOT}")
target_link_libraries(Physics PRIVATE Jolt)

# The bridge header switches to dllimport for consumers, so the payload's own build states that it
# is the exporting side.
target_compile_definitions(Physics PRIVATE PHYSICS_EXPORTS)

set_target_properties(Physics PROPERTIES
    FOLDER "Physics"
)

# The package's own gate is built only by a test-host run: the generated top-level CMake derives
# ARISEN_TEST_HOST from the profile's test-host role, so product profiles never compile the test DLL.
if(ARISEN_TEST_HOST)
    add_subdirectory(Tests)
endif()
