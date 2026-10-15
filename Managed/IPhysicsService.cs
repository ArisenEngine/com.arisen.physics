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

    /// <summary>True when any body overlaps the sphere, the melee-style query combat reads.</summary>
    bool TryOverlapSphere(PhysicsWorldHandle world, PhysicsVec3 center, float radius);

    /// <summary>
    /// Creates a kinematic character capsule whose base (feet) is at <paramref name="feetPosition"/>.
    /// The solver slides it along geometry, steps it up stairs, and keeps it on walkable ground.
    /// </summary>
    PhysicsCharacterHandle CreateCharacter(
        PhysicsWorldHandle world,
        float radius,
        float halfHeightOfCylinder,
        PhysicsVec3 feetPosition);

    void DestroyCharacter(PhysicsWorldHandle world, PhysicsCharacterHandle character);

    /// <summary>
    /// Advances one character by <paramref name="deltaTime"/>. The horizontal displacement is
    /// sticky - blocked geometry redirects it instead of stopping it - and gravity is integrated
    /// while the character is airborne.
    /// </summary>
    bool MoveCharacter(
        PhysicsWorldHandle world,
        PhysicsCharacterHandle character,
        float displacementX,
        float displacementZ,
        float deltaTime,
        out PhysicsVec3 feetPosition,
        out bool grounded);

    bool TryGetCharacterPosition(
        PhysicsWorldHandle world,
        PhysicsCharacterHandle character,
        out PhysicsVec3 feetPosition);

    /// <summary>True while the solver reports the character standing on walkable ground.</summary>
    bool IsCharacterGrounded(PhysicsWorldHandle world, PhysicsCharacterHandle character);

    /// <summary>Launches a grounded character upward; false while it is airborne.</summary>
    bool TryJumpCharacter(PhysicsWorldHandle world, PhysicsCharacterHandle character, float speed);

    /// <summary>Characters alive in this world, so a gate can see a leak.</summary>
    int GetCharacterCount(PhysicsWorldHandle world);
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

/// <summary>A character owned by a world. A negative handle is not valid.</summary>
public readonly record struct PhysicsCharacterHandle(int Value)
{
    public bool IsValid => Value >= 0;
}

/// <summary>Plain-data vector at the boundary; the solver's own types never cross it.</summary>
public readonly record struct PhysicsVec3(float X, float Y, float Z);

/// <summary>Plain-data quaternion at the boundary, in x/y/z/w order.</summary>
public readonly record struct PhysicsQuat(float X, float Y, float Z, float W);
