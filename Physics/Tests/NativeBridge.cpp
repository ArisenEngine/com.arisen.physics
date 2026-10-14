#include "PhysicsGateTests.h"

// The package's gate bridge: the test host loads this library by name, calls the single registration
// export, and receives one entry per case. Each entry runs the case through the same flat C ABI the
// managed service binds to, so a pass means the boundary works, not only the solver.

#ifdef _WIN32
#define ARISEN_PHYSICS_TEST_EXPORT __declspec(dllexport)
#else
#define ARISEN_PHYSICS_TEST_EXPORT __attribute__((visibility("default")))
#endif

using namespace ArisenEngine::Testing;

namespace
{
    bool RunRegisteredTest(const char* name)
    {
        const auto results = TestRunner::RunTestByName(name);
        return results.size() == 1 && results[0].passed;
    }
}

extern "C"
{
    ARISEN_PHYSICS_TEST_EXPORT void RegisterNativeTests(void (*registerCallback)(const char*, bool (*)()))
    {
        // 1. Register the cases in this library's own registry.
        TestRunner::RegisterTest<PhysicsDeterminismTest>();
        TestRunner::RegisterTest<PhysicsBodyOwnershipTest>();
        TestRunner::RegisterTest<PhysicsWorldDrainTest>();

        // 2. Wrap them for the managed test runner, which calls each entry by name.
        registerCallback("Physics.Unit.PhysicsDeterminismTest", []()
        {
            return RunRegisteredTest("PhysicsDeterminismTest");
        });
        registerCallback("Physics.Unit.PhysicsBodyOwnershipTest", []()
        {
            return RunRegisteredTest("PhysicsBodyOwnershipTest");
        });
        registerCallback("Physics.Unit.PhysicsWorldDrainTest", []()
        {
            return RunRegisteredTest("PhysicsWorldDrainTest");
        });
    }
}
