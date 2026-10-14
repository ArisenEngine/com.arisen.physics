namespace ArisenEngine.Physics;

/// <summary>
/// The boundary gameplay packages consume. Solver types never appear here: bodies are integer handles
/// the simulation owns, and every query returns plain data.
///
/// D1's first slice carries what D2 and D3 extend: a world stepped in fixed increments, static boxes
/// and character capsules, body positions, and a closest-hit ray query. The service is registered by
/// the package, so a gameplay package resolves it through <c>IServiceRegistry</c> once and caches it.
/// </summary>
public interface IPhysicsService
{
    /// <summary>The vendored solver's version, for diagnostics and gates.</summary>
    string SolverVersion { get; }

    /// <summary>
    /// Creates a world with the default gravity and a fixed 1/60 s step. The caller steps it, so a
    /// replay of the same inputs produces the same states.
    /// </summary>
    PhysicsWorldHandle CreateWorld();

    void DestroyWorld(PhysicsWorldHandle world);

    /// <summary>Adds a static axis-aligned box and returns its body handle.</summary>
    PhysicsBodyHandle AddStaticBox(
        PhysicsWorldHandle world,
        float halfExtentX,
        float halfExtentY,
        float halfExtentZ,
        float positionX,
        float positionY,
        float positionZ);

    /// <summary>Adds a dynamic capsule, the shape the character controller builds on.</summary>
    PhysicsBodyHandle AddCapsule(
        PhysicsWorldHandle world,
        float radius,
        float halfHeightOfCylinder,
        float positionX,
        float positionY,
        float positionZ);

    /// <summary>Adds a dynamic box with the given friction.</summary>
    PhysicsBodyHandle AddDynamicBox(
        PhysicsWorldHandle world,
        PhysicsVec3 halfExtents,
        PhysicsVec3 position,
        float friction);

    /// <summary>Adds a static capsule.</summary>
    PhysicsBodyHandle AddStaticCapsule(
        PhysicsWorldHandle world,
        float radius,
        float halfHeightOfCylinder,
        float positionX,
        float positionY,
        float positionZ);

    /// <summary>Advances the world by fixed steps. Returns false when the world is not valid.</summary>
    bool Step(PhysicsWorldHandle world, int stepCount);

    /// <summary>Bodies the world currently owns.</summary>
    int GetBodyCount(PhysicsWorldHandle world);

    /// <summary>Removes a body the world owns. Returns false for an unknown handle.</summary>
    bool RemoveBody(PhysicsWorldHandle world, PhysicsBodyHandle body);

    /// <summary>Worlds that exist right now, so a leak is visible without inspecting the solver.</summary>
    int LiveWorldCount { get; }

    /// <summary>Reads a body's world position.</summary>
    bool TryGetBodyPosition(PhysicsWorldHandle world, PhysicsBodyHandle body, out PhysicsVec3 position);

    /// <summary>Reads a body's world rotation.</summary>
    bool TryGetBodyRotation(PhysicsWorldHandle world, PhysicsBodyHandle body, out PhysicsQuat rotation);

    /// <summary>Sets a body's rotation, which is how an authored pose is applied after creation.</summary>
    bool TrySetBodyRotation(PhysicsWorldHandle world, PhysicsBodyHandle body, PhysicsQuat rotation);

    /// <summary>Casts a ray and reports the closest hit distance along the normalized direction.</summary>
    bool TryRayCastClosest(
        PhysicsWorldHandle world,
        PhysicsVec3 origin,
        PhysicsVec3 direction,
        float maxDistance,
        out float distance);
}

/// <summary>An owned simulation world. A default handle is not valid.</summary>
public readonly record struct PhysicsWorldHandle(nint Value)
{
    public bool IsValid => Value != 0;
}

/// <summary>A body owned by a world. A negative handle is not valid.</summary>
public readonly record struct PhysicsBodyHandle(int Value)
{
    public bool IsValid => Value >= 0;
}

/// <summary>Plain-data vector at the boundary; the solver's own types never cross it.</summary>
public readonly record struct PhysicsVec3(float X, float Y, float Z);

/// <summary>Plain-data quaternion at the boundary, in x/y/z/w order.</summary>
public readonly record struct PhysicsQuat(float X, float Y, float Z, float W);
