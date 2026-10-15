#include "PhysicsBridge.h"

#include <cstdio>
#include <cstring>
#include <atomic>
#include <memory>
#include <vector>

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

namespace
{
    using namespace JPH;

    // Two object layers cover the boundary's first slice: the room's static geometry and the moving
    // characters. D2/D3 extend this when authored collision arrives.
    namespace Layers
    {
        constexpr ObjectLayer NonMoving = 0;
        constexpr ObjectLayer Moving = 1;
        constexpr ObjectLayer Count = 2;
    }

    namespace BroadPhaseLayers
    {
        constexpr BroadPhaseLayer NonMoving(0);
        constexpr BroadPhaseLayer Moving(1);
        constexpr unsigned int Count = 2;
    }

    class BroadPhaseLayerInterfaceImpl final : public BroadPhaseLayerInterface
    {
    public:
        unsigned int GetNumBroadPhaseLayers() const override
        {
            return BroadPhaseLayers::Count;
        }

        BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer layer) const override
        {
            return layer == Layers::Moving ? BroadPhaseLayers::Moving : BroadPhaseLayers::NonMoving;
        }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
        const char* GetBroadPhaseLayerName(BroadPhaseLayer layer) const override
        {
            return layer == BroadPhaseLayers::Moving ? "Moving" : "NonMoving";
        }
#endif
    };

    class ObjectVsBroadPhaseLayerFilterImpl final : public ObjectVsBroadPhaseLayerFilter
    {
    public:
        bool ShouldCollide(ObjectLayer layer1, BroadPhaseLayer layer2) const override
        {
            if (layer1 == Layers::NonMoving)
            {
                return layer2 == BroadPhaseLayers::Moving;
            }

            return true;
        }
    };

    class ObjectLayerPairFilterImpl final : public ObjectLayerPairFilter
    {
    public:
        bool ShouldCollide(ObjectLayer layer1, ObjectLayer layer2) const override
        {
            if (layer1 == Layers::NonMoving)
            {
                return layer2 == Layers::Moving;
            }

            if (layer1 == Layers::Moving)
            {
                return true;
            }

            return false;
        }
    };

    // Fixed-step, single-threaded solver configuration: identical inputs produce identical state, and
    // nothing steps the world behind the caller's back.
    constexpr float FixedTimeStep = 1.0f / 60.0f;
    constexpr float GravityY = -9.81f;
    constexpr int MaxBodies = 1024;
    constexpr int MaxBodyPairs = 1024;
    constexpr int MaxContactConstraints = 1024;
    constexpr unsigned int TempAllocatorBytes = 10u * 1024u * 1024u;
    constexpr int MaxHandles = 4096;

    std::atomic<int> s_LiveWorlds { 0 };

    // Characters move through the solver but are not rigid bodies: the caller sets a velocity and
    // the solver slides, steps, and sticks them.
    struct Character
    {
        RefConst<Shape> Shape;
        std::unique_ptr<CharacterVirtual> Virtual;
    };

    struct World
    {
        TempAllocatorImpl TempAllocator { TempAllocatorBytes };
        JobSystemThreadPool JobSystem { cMaxPhysicsJobs, cMaxPhysicsBarriers, 1 };
        BroadPhaseLayerInterfaceImpl BroadPhaseLayers;
        ObjectVsBroadPhaseLayerFilterImpl ObjectVsBroadPhaseLayerFilter;
        ObjectLayerPairFilterImpl ObjectLayerPairFilter;
        PhysicsSystem System;
        std::vector<BodyID> Bodies;
        std::vector<std::unique_ptr<Character>> Characters;
        int NextHandle = 0;
        bool Initialized = false;

        World()
        {
            System.Init(
                MaxBodies,
                0,
                MaxBodyPairs,
                MaxContactConstraints,
                BroadPhaseLayers,
                ObjectVsBroadPhaseLayerFilter,
                ObjectLayerPairFilter);
            System.SetGravity(Vec3(0.0f, GravityY, 0.0f));
            Bodies.resize(MaxHandles, BodyID());
            Initialized = true;
            s_LiveWorlds.fetch_add(1, std::memory_order_relaxed);
        }

        ~World()
        {
            // The system's bodies and the job system's threads are released with the world, which is
            // what makes a destroyed world leave nothing behind.
            s_LiveWorlds.fetch_sub(1, std::memory_order_relaxed);
        }

        int AddBody(const BodyCreationSettings& settings)
        {
            for (int handle = 0; handle < MaxHandles; handle++)
            {
                int candidate = (NextHandle + handle) % MaxHandles;
                if (!Bodies[candidate].IsInvalid())
                {
                    continue;
                }

                BodyID body = System.GetBodyInterface().CreateAndAddBody(settings, EActivation::Activate);
                if (body.IsInvalid())
                {
                    return -1;
                }

                Bodies[candidate] = body;
                NextHandle = (candidate + 1) % MaxHandles;
                return candidate;
            }

            return -1;
        }
    };

    World* AsWorld(void* world)
    {
        return static_cast<World*>(world);
    }

    // The solver's allocator and type factory are process-wide and have to exist before the first
    // world allocates anything, so the entry point that creates a world installs them once.
    void EnsureSolverInitialized()
    {
        static bool initialized = false;
        if (initialized)
        {
            return;
        }

        RegisterDefaultAllocator();
        Factory::sInstance = new Factory();
        RegisterTypes();
        initialized = true;
    }
}

int Physics_GetSolverVersion(char* buffer, int capacity)
{
    char text[64];
    int length = std::snprintf(
        text,
        sizeof(text),
        "%d.%d.%d",
        JPH_VERSION_MAJOR,
        JPH_VERSION_MINOR,
        JPH_VERSION_PATCH);
    if (capacity < length + 1)
    {
        return -1;
    }

    if (buffer != nullptr)
    {
        std::memcpy(buffer, text, static_cast<size_t>(length) + 1);
    }

    return length;
}

void* Physics_CreateWorld()
{
    EnsureSolverInitialized();
    return new World();
}

void Physics_DestroyWorld(void* world)
{
    delete AsWorld(world);
}

int Physics_AddStaticBox(
    void* world,
    float halfExtentX,
    float halfExtentY,
    float halfExtentZ,
    float positionX,
    float positionY,
    float positionZ)
{
    World* target = AsWorld(world);
    if (target == nullptr)
    {
        return -1;
    }

    BodyCreationSettings settings(
        new BoxShape(Vec3(halfExtentX, halfExtentY, halfExtentZ)),
        RVec3(positionX, positionY, positionZ),
        Quat::sIdentity(),
        EMotionType::Static,
        Layers::NonMoving);
    return target->AddBody(settings);
}

int Physics_AddCapsule(
    void* world,
    float radius,
    float halfHeightOfCylinder,
    float positionX,
    float positionY,
    float positionZ)
{
    World* target = AsWorld(world);
    if (target == nullptr)
    {
        return -1;
    }

    BodyCreationSettings settings(
        new CapsuleShape(halfHeightOfCylinder, radius),
        RVec3(positionX, positionY, positionZ),
        Quat::sIdentity(),
        EMotionType::Dynamic,
        Layers::Moving);
    settings.mFriction = 0.5f;
    return target->AddBody(settings);
}

int Physics_AddDynamicBox(
    void* world,
    float halfExtentX,
    float halfExtentY,
    float halfExtentZ,
    float positionX,
    float positionY,
    float positionZ,
    float friction)
{
    World* target = AsWorld(world);
    if (target == nullptr)
    {
        return -1;
    }

    BodyCreationSettings settings(
        new BoxShape(Vec3(halfExtentX, halfExtentY, halfExtentZ)),
        RVec3(positionX, positionY, positionZ),
        Quat::sIdentity(),
        EMotionType::Dynamic,
        Layers::Moving);
    settings.mFriction = friction;
    return target->AddBody(settings);
}

int Physics_AddStaticCapsule(
    void* world,
    float radius,
    float halfHeightOfCylinder,
    float positionX,
    float positionY,
    float positionZ)
{
    World* target = AsWorld(world);
    if (target == nullptr)
    {
        return -1;
    }

    BodyCreationSettings settings(
        new CapsuleShape(halfHeightOfCylinder, radius),
        RVec3(positionX, positionY, positionZ),
        Quat::sIdentity(),
        EMotionType::Static,
        Layers::NonMoving);
    return target->AddBody(settings);
}

int Physics_Step(void* world, int stepCount)
{
    World* target = AsWorld(world);
    if (target == nullptr || !target->Initialized || stepCount <= 0)
    {
        return 0;
    }

    // Jolt's Update splits one frame delta into `inCollisionSteps` collision sub-steps, so the fixed
    // step is the delta: `stepCount` fixed steps are `stepCount` updates of one sub-step each. Passing
    // the count as the sub-step count instead would advance a single 1/60 s frame, which is why the
    // first gate read a capsule that had barely moved.
    for (int step = 0; step < stepCount; step++)
    {
        target->System.Update(FixedTimeStep, 1, &target->TempAllocator, &target->JobSystem);
    }

    return 1;
}

int Physics_GetBodyCount(void* world)
{
    World* target = AsWorld(world);
    return target == nullptr ? 0 : static_cast<int>(target->System.GetNumBodies());
}

int Physics_RemoveBody(void* world, int body)
{
    World* target = AsWorld(world);
    if (target == nullptr || body < 0 || body >= MaxHandles)
    {
        return 0;
    }

    BodyID id = target->Bodies[body];
    if (id.IsInvalid())
    {
        return 0;
    }

    BodyInterface& bodies = target->System.GetBodyInterface();
    bodies.RemoveBody(id);
    bodies.DestroyBody(id);
    target->Bodies[body] = BodyID();
    return 1;
}

int Physics_GetLiveWorldCount()
{
    return s_LiveWorlds.load(std::memory_order_relaxed);
}

int Physics_GetBodyPosition(void* world, int body, float* outXyz)
{
    World* target = AsWorld(world);
    if (target == nullptr || outXyz == nullptr || body < 0 || body >= MaxHandles)
    {
        return 0;
    }

    BodyID id = target->Bodies[body];
    if (id.IsInvalid())
    {
        return 0;
    }

    RVec3 position = target->System.GetBodyInterface().GetPosition(id);
    outXyz[0] = static_cast<float>(position.GetX());
    outXyz[1] = static_cast<float>(position.GetY());
    outXyz[2] = static_cast<float>(position.GetZ());
    return 1;
}

int Physics_GetBodyRotation(void* world, int body, float* outXyzw)
{
    World* target = AsWorld(world);
    if (target == nullptr || outXyzw == nullptr || body < 0 || body >= MaxHandles)
    {
        return 0;
    }

    BodyID id = target->Bodies[body];
    if (id.IsInvalid())
    {
        return 0;
    }

    Quat rotation = target->System.GetBodyInterface().GetRotation(id);
    outXyzw[0] = rotation.GetX();
    outXyzw[1] = rotation.GetY();
    outXyzw[2] = rotation.GetZ();
    outXyzw[3] = rotation.GetW();
    return 1;
}

int Physics_SetBodyRotation(void* world, int body, float x, float y, float z, float w)
{
    World* target = AsWorld(world);
    if (target == nullptr || body < 0 || body >= MaxHandles)
    {
        return 0;
    }

    BodyID id = target->Bodies[body];
    if (id.IsInvalid())
    {
        return 0;
    }

    Quat rotation(x, y, z, w);
    if (rotation.LengthSq() <= 0.0f)
    {
        return 0;
    }

    target->System.GetBodyInterface().SetRotation(id, rotation.Normalized(), EActivation::DontActivate);
    return 1;
}

int Physics_RayCastClosest(
    void* world,
    const float* originXyz,
    const float* directionXyz,
    float maxDistance,
    float* outDistance)
{
    World* target = AsWorld(world);
    if (target == nullptr || originXyz == nullptr || directionXyz == nullptr || outDistance == nullptr)
    {
        return 0;
    }

    Vec3 direction(directionXyz[0], directionXyz[1], directionXyz[2]);
    if (direction.LengthSq() <= 0.0f)
    {
        return 0;
    }

    direction = direction.Normalized() * maxDistance;
    RRayCast ray(RVec3(originXyz[0], originXyz[1], originXyz[2]), direction);
    RayCastResult hit;
    if (!target->System.GetNarrowPhaseQuery().CastRay(ray, hit))
    {
        return 0;
    }

    *outDistance = hit.mFraction * maxDistance;
    return 1;
}

namespace
{
    // Stops as soon as one overlap is found: a hit/hurt query asks whether anything is there, and
    // the answer must not cost a full traversal of everything that touches the sphere.
    class OverlapAnyCollector final : public CollideShapeCollector
    {
    public:
        void AddHit(const ResultType& inResult) override
        {
            (void)inResult;
            mHadHit = true;
            ForceEarlyOut();
        }

        bool HadHit() const { return mHadHit; }

    private:
        bool mHadHit = false;
    };
}

int Physics_OverlapSphere(void* world, float centerX, float centerY, float centerZ, float radius)
{
    World* target = AsWorld(world);
    if (target == nullptr || !(radius > 0.0f))
    {
        return 0;
    }

    SphereShape sphere(radius);
    CollideShapeSettings settings;
    OverlapAnyCollector collector;
    target->System.GetNarrowPhaseQuery().CollideShape(
        &sphere,
        Vec3::sOne(),
        RMat44::sTranslation(RVec3(centerX, centerY, centerZ)),
        settings,
        RVec3(centerX, centerY, centerZ),
        collector);
    return collector.HadHit() ? 1 : 0;
}

int Physics_CreateCharacter(
    void* world,
    float radius,
    float halfHeightOfCylinder,
    float positionX,
    float positionY,
    float positionZ)
{
    World* target = AsWorld(world);
    if (target == nullptr || !(radius > 0.0f) || halfHeightOfCylinder < 0.0f)
    {
        return -1;
    }

    auto character = std::make_unique<Character>();
    // The character shape must have its bottom at the local origin: the character's position is
    // its base (feet), and the capsule sits above that origin.
    character->Shape = new RotatedTranslatedShape(
        Vec3(0.0f, halfHeightOfCylinder + radius, 0.0f),
        Quat::sIdentity(),
        new CapsuleShape(halfHeightOfCylinder, radius));
    CharacterVirtualSettings settings;
    settings.mShape = character->Shape;
    character->Virtual = std::make_unique<CharacterVirtual>(
        &settings,
        RVec3(positionX, positionY, positionZ),
        Quat::sIdentity(),
        0,
        &target->System);

    for (int handle = 0; handle < static_cast<int>(target->Characters.size()); handle++)
    {
        if (!target->Characters[handle])
        {
            target->Characters[handle] = std::move(character);
            return handle;
        }
    }

    target->Characters.push_back(std::move(character));
    return static_cast<int>(target->Characters.size()) - 1;
}

void Physics_DestroyCharacter(void* world, int character)
{
    World* target = AsWorld(world);
    if (target == nullptr || character < 0 || character >= static_cast<int>(target->Characters.size()))
    {
        return;
    }

    target->Characters[character].reset();
}

int Physics_MoveCharacter(
    void* world,
    int character,
    float displacementX,
    float displacementZ,
    float deltaTime,
    float* outXyz,
    int* outGrounded)
{
    World* target = AsWorld(world);
    if (target == nullptr || outXyz == nullptr || outGrounded == nullptr || deltaTime <= 0.0f ||
        character < 0 || character >= static_cast<int>(target->Characters.size()) ||
        !target->Characters[character])
    {
        return 0;
    }

    CharacterVirtual* virtual_character = target->Characters[character]->Virtual.get();
    Vec3 velocity = virtual_character->GetLinearVelocity();
    velocity.SetX(displacementX / deltaTime);
    velocity.SetZ(displacementZ / deltaTime);

    bool grounded = virtual_character->GetGroundState() == CharacterBase::EGroundState::OnGround;
    velocity.SetY(velocity.GetY() + GravityY * deltaTime);
    if (grounded && velocity.GetY() < 0.0f)
    {
        // Grounded: the solver's stick-to-floor keeps the character attached, and the vertical
        // speed only carries its per-frame gravity increment. A continuous downward velocity
        // pushes into the floor instead, which loses the grounded state and with it stair walking.
        velocity.SetY(GravityY * deltaTime);
    }

    virtual_character->SetLinearVelocity(velocity);
    CharacterVirtual::ExtendedUpdateSettings update_settings;
    virtual_character->ExtendedUpdate(
        deltaTime,
        Vec3(0.0f, GravityY, 0.0f),
        update_settings,
        BroadPhaseLayerFilter(),
        ObjectLayerFilter(),
        BodyFilter(),
        ShapeFilter(),
        target->TempAllocator);

    RVec3 position = virtual_character->GetPosition();
    outXyz[0] = static_cast<float>(position.GetX());
    outXyz[1] = static_cast<float>(position.GetY());
    outXyz[2] = static_cast<float>(position.GetZ());
    *outGrounded = virtual_character->GetGroundState() == CharacterBase::EGroundState::OnGround ? 1 : 0;
    return 1;
}

int Physics_GetCharacterPosition(void* world, int character, float* outXyz)
{
    World* target = AsWorld(world);
    if (target == nullptr || outXyz == nullptr ||
        character < 0 || character >= static_cast<int>(target->Characters.size()) ||
        !target->Characters[character])
    {
        return 0;
    }

    RVec3 position = target->Characters[character]->Virtual->GetPosition();
    outXyz[0] = static_cast<float>(position.GetX());
    outXyz[1] = static_cast<float>(position.GetY());
    outXyz[2] = static_cast<float>(position.GetZ());
    return 1;
}

int Physics_GetCharacterGroundState(void* world, int character)
{
    World* target = AsWorld(world);
    if (target == nullptr ||
        character < 0 || character >= static_cast<int>(target->Characters.size()) ||
        !target->Characters[character])
    {
        return 0;
    }

    switch (target->Characters[character]->Virtual->GetGroundState())
    {
    case CharacterBase::EGroundState::OnGround:
        return 1;
    case CharacterBase::EGroundState::OnSteepGround:
        return 2;
    case CharacterBase::EGroundState::NotSupported:
        return 3;
    case CharacterBase::EGroundState::InAir:
        return 4;
    default:
        return 0;
    }
}

int Physics_GetCharacterCount(void* world)
{
    World* target = AsWorld(world);
    if (target == nullptr)
    {
        return 0;
    }

    int count = 0;
    for (const std::unique_ptr<Character>& character : target->Characters)
    {
        if (character)
        {
            count++;
        }
    }

    return count;
}
