# The package's own gate. The test host builds this target alone (ARISEN_TEST_HOST) and discovers it
# through the package's `nativeTests`, so the library is a build output of that run rather than a
# shipped payload.

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(Arisen.Physics.Test SHARED
    NativeBridge.cpp
    PhysicsGateTests.h
)

# Framework/TestRunner.h resolves Base/FoundationMinimal.h and Logger/Logger.h through the include
# directories Core.Foundation and Core.Diagnostic publish.
target_link_libraries(Arisen.Physics.Test PRIVATE
    Core.Foundation
    Core.Diagnostic
    Physics
)

set_target_properties(Arisen.Physics.Test PROPERTIES
    FOLDER "Physics"
)
