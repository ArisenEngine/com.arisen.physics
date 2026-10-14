#pragma once

// The package's own gate. Every case goes through the flat C ABI the managed service binds to, so the
// cases fail when the boundary breaks, not only when the solver does.

#include "Framework/TestRunner.h"
#include "../PhysicsBridge.h"

#include <cmath>
#include <cstring>

namespace ArisenEngine::Testing
{
    namespace PhysicsGate
    {
        constexpr float CapsuleRadius = 0.35f;
        constexpr float CapsuleHalfHeight = 0.8f;
        constexpr float CapsuleStartY = 4.0f;
        constexpr int DeterminismSteps = 240;

        /// The canonical scene of D1's gate: a floor at the origin and one character capsule above it.
        inline bool BuildCanonicalScene(void* world, int& capsule)
        {
            if (Physics_AddStaticBox(world, 20.0f, 0.5f, 20.0f, 0.0f, -0.5f, 0.0f) < 0)
            {
                return false;
            }

            capsule = Physics_AddCapsule(world, CapsuleRadius, CapsuleHalfHeight, 0.0f, CapsuleStartY, 0.0f);
            return capsule >= 0;
        }

        inline bool ReadCapsuleHeight(void* world, int capsule, float& y)
        {
            float position[3] = { 0.0f, 0.0f, 0.0f };
            if (Physics_GetBodyPosition(world, capsule, position) == 0)
            {
                return false;
            }

            y = position[1];
            return true;
        }
    }

    /// The same inputs must produce the same final state, so a replay of a slice is reproducible. A
    /// second world steps the identical scene and both capsules have to land on identical bits, which
    /// is only meaningful because the solver is stepped by the caller in fixed increments.
    class PhysicsDeterminismTest : public ITest
    {
    public:
        const char* GetName() const override { return "PhysicsDeterminismTest"; }
        TestCategory GetCategory() const override { return TestCategory::Unit; }

        bool Setup() override { return true; }

        bool Run() override
        {
            void* first = Physics_CreateWorld();
            void* second = Physics_CreateWorld();
            if (first == nullptr || second == nullptr)
            {
                LOG_ERROR("Physics gate: a world could not be created.");
                Physics_DestroyWorld(first);
                Physics_DestroyWorld(second);
                return false;
            }

            int firstCapsule = -1;
            int secondCapsule = -1;
            bool built = PhysicsGate::BuildCanonicalScene(first, firstCapsule) &&
                PhysicsGate::BuildCanonicalScene(second, secondCapsule);
            if (!built)
            {
                LOG_ERROR("Physics gate: the canonical scene could not be built.");
                Physics_DestroyWorld(first);
                Physics_DestroyWorld(second);
                return false;
            }

            bool stepped = Physics_Step(first, PhysicsGate::DeterminismSteps) != 0 &&
                Physics_Step(second, PhysicsGate::DeterminismSteps) != 0;

            float firstHeight = 0.0f;
            float secondHeight = 0.0f;
            bool read = PhysicsGate::ReadCapsuleHeight(first, firstCapsule, firstHeight) &&
                PhysicsGate::ReadCapsuleHeight(second, secondCapsule, secondHeight);

            // The capsule has to have fallen and to have been stopped by the floor, so the comparison
            // is not between two capsules that never moved.
            bool landed = firstHeight < PhysicsGate::CapsuleStartY - 1.0f &&
                firstHeight > PhysicsGate::CapsuleHalfHeight;

            bool identical = std::memcmp(&firstHeight, &secondHeight, sizeof(float)) == 0;

            // The query D3 reads: a ray straight down from above the floor finds it.
            float hitDistance = 0.0f;
            const float origin[3] = { 0.0f, 10.0f, 0.0f };
            const float direction[3] = { 0.0f, -1.0f, 0.0f };
            bool hit = Physics_RayCastClosest(first, origin, direction, 20.0f, &hitDistance) != 0;

            Physics_DestroyWorld(first);
            Physics_DestroyWorld(second);

            if (!stepped || !read || !landed || !identical || !hit)
            {
                LOG_ERROR(
                    String::Format(
                        "Physics gate: stepped=%d read=%d landed=%d identical=%d rayHit=%d",
                        stepped ? 1 : 0,
                        read ? 1 : 0,
                        landed ? 1 : 0,
                        identical ? 1 : 0,
                        hit ? 1 : 0).c_str());
                return false;
            }

            LOG_INFO(
                String::Format(
                    "Physics determinism: two worlds stepped %d fixed steps agree on %f, floor hit at %f.",
                    PhysicsGate::DeterminismSteps,
                    static_cast<double>(firstHeight),
                    static_cast<double>(hitDistance)).c_str());
            return true;
        }

        void Teardown() override { }
    };

    /// Bodies are owned by the simulation: the world counts them, a removed body stops answering, and
    /// another world never sees it.
    class PhysicsBodyOwnershipTest : public ITest
    {
    public:
        const char* GetName() const override { return "PhysicsBodyOwnershipTest"; }
        TestCategory GetCategory() const override { return TestCategory::Unit; }

        bool Setup() override { return true; }

        bool Run() override
        {
            int worldsBefore = Physics_GetLiveWorldCount();
            void* world = Physics_CreateWorld();
            if (world == nullptr)
            {
                LOG_ERROR("Physics gate: a world could not be created.");
                return false;
            }

            int capsule = -1;
            bool built = PhysicsGate::BuildCanonicalScene(world, capsule);
            int second = Physics_AddCapsule(world, 0.3f, 0.7f, 2.0f, 3.0f, 0.0f);
            int countAfterCreate = Physics_GetBodyCount(world);

            bool removed = Physics_RemoveBody(world, second) != 0;
            int countAfterRemove = Physics_GetBodyCount(world);
            float position[3] = { 0.0f, 0.0f, 0.0f };
            bool removedBodyIsGone = Physics_GetBodyPosition(world, second, position) == 0;
            bool capsuleSurvives = Physics_GetBodyPosition(world, capsule, position) != 0;

            Physics_DestroyWorld(world);
            int worldsAfter = Physics_GetLiveWorldCount();

            if (!built || countAfterCreate != 3 || !removed || countAfterRemove != 2 ||
                !removedBodyIsGone || !capsuleSurvives || worldsAfter != worldsBefore)
            {
                LOG_ERROR(
                    String::Format(
                        "Physics gate: built=%d afterCreate=%d removed=%d afterRemove=%d gone=%d "
                        "capsule=%d worlds=%d/%d",
                        built ? 1 : 0,
                        countAfterCreate,
                        removed ? 1 : 0,
                        countAfterRemove,
                        removedBodyIsGone ? 1 : 0,
                        capsuleSurvives ? 1 : 0,
                        worldsAfter,
                        worldsBefore).c_str());
                return false;
            }

            LOG_INFO("Physics body ownership: counts, removal, and a fresh world all agree.");
            return true;
        }

        void Teardown() override { }
    };

    /// A destroyed world leaves nothing behind: repeating a full life cycle neither leaks a world nor
    /// leaves the next one seeded with the previous scene, and every step finishes before the destroy
    /// returns.
    class PhysicsWorldDrainTest : public ITest
    {
    public:
        const char* GetName() const override { return "PhysicsWorldDrainTest"; }
        TestCategory GetCategory() const override { return TestCategory::Unit; }

        bool Setup() override { return true; }

        bool Run() override
        {
            constexpr int CycleCount = 3;
            int worldsBefore = Physics_GetLiveWorldCount();
            for (int cycle = 0; cycle < CycleCount; cycle++)
            {
                void* world = Physics_CreateWorld();
                if (world == nullptr || Physics_GetBodyCount(world) != 0)
                {
                    LOG_ERROR("Physics gate: a new world is not empty.");
                    Physics_DestroyWorld(world);
                    return false;
                }

                int capsule = -1;
                if (!PhysicsGate::BuildCanonicalScene(world, capsule) ||
                    Physics_Step(world, PhysicsGate::DeterminismSteps) == 0)
                {
                    LOG_ERROR("Physics gate: a world life cycle did not complete.");
                    Physics_DestroyWorld(world);
                    return false;
                }

                Physics_DestroyWorld(world);
                if (Physics_GetLiveWorldCount() != worldsBefore)
                {
                    LOG_ERROR("Physics gate: a destroyed world is still counted.");
                    return false;
                }
            }

            LOG_INFO(String::Format("Physics world drain: %d life cycles left no world behind.", CycleCount).c_str());
            return true;
        }

        void Teardown() override { }
    };
}
