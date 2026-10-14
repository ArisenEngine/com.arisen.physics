# The package payload: one shared library with a flat C ABI that the managed service binds to. Solver
# types stay behind this boundary, so no gameplay or engine package ever includes a Jolt header.
add_library(Physics SHARED
    PhysicsBridge.cpp
    PhysicsBridge.h
)

target_include_directories(Physics PRIVATE "${PHYSICS_REPO_ROOT}")
target_link_libraries(Physics PRIVATE Jolt)

set_target_properties(Physics PROPERTIES
    FOLDER "Physics"
)
