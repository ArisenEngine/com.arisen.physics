using System.Runtime.InteropServices;

namespace ArisenEngine.Physics;

/// <summary>
/// Binds the package payload's flat C ABI. Every entry point mirrors `Physics/PhysicsBridge.h`; the
/// solver's types stay on the other side of it.
/// </summary>
internal sealed class PhysicsService : IPhysicsService
{
    private const string Library = "Physics";

    public string SolverVersion { get; }

    public PhysicsService()
    {
        Span<byte> buffer = stackalloc byte[64];
        int length = Physics_GetSolverVersion(ref MemoryMarshal.GetReference(buffer), buffer.Length);
        if (length <= 0)
        {
            throw new InvalidOperationException(
                "[PhysicsService] The physics payload did not report its solver version.");
        }

        SolverVersion = System.Text.Encoding.ASCII.GetString(buffer[..length]);
    }

    public PhysicsWorldHandle CreateWorld() => new(Physics_CreateWorld());

    public void DestroyWorld(PhysicsWorldHandle world)
    {
        if (world.IsValid)
        {
            Physics_DestroyWorld(world.Value);
        }
    }

    public PhysicsBodyHandle AddStaticBox(
        PhysicsWorldHandle world,
        float halfExtentX,
        float halfExtentY,
        float halfExtentZ,
        float positionX,
        float positionY,
        float positionZ) =>
        new(Physics_AddStaticBox(
            world.Value,
            halfExtentX,
            halfExtentY,
            halfExtentZ,
            positionX,
            positionY,
            positionZ));

    public PhysicsBodyHandle AddCapsule(
        PhysicsWorldHandle world,
        float radius,
        float halfHeightOfCylinder,
        float positionX,
        float positionY,
        float positionZ) =>
        new(Physics_AddCapsule(
            world.Value,
            radius,
            halfHeightOfCylinder,
            positionX,
            positionY,
            positionZ));

    public PhysicsBodyHandle AddDynamicBox(
        PhysicsWorldHandle world,
        PhysicsVec3 halfExtents,
        PhysicsVec3 position,
        float friction) =>
        new(Physics_AddDynamicBox(
            world.Value,
            halfExtents.X,
            halfExtents.Y,
            halfExtents.Z,
            position.X,
            position.Y,
            position.Z,
            friction));

    public PhysicsBodyHandle AddStaticCapsule(
        PhysicsWorldHandle world,
        float radius,
        float halfHeightOfCylinder,
        float positionX,
        float positionY,
        float positionZ) =>
        new(Physics_AddStaticCapsule(
            world.Value,
            radius,
            halfHeightOfCylinder,
            positionX,
            positionY,
            positionZ));

    public bool Step(PhysicsWorldHandle world, int stepCount) =>
        world.IsValid && Physics_Step(world.Value, stepCount) != 0;

    public int GetBodyCount(PhysicsWorldHandle world) =>
        world.IsValid ? Physics_GetBodyCount(world.Value) : 0;

    public bool RemoveBody(PhysicsWorldHandle world, PhysicsBodyHandle body) =>
        world.IsValid && body.IsValid && Physics_RemoveBody(world.Value, body.Value) != 0;

    public int LiveWorldCount => Physics_GetLiveWorldCount();

    public bool TryGetBodyPosition(
        PhysicsWorldHandle world,
        PhysicsBodyHandle body,
        out PhysicsVec3 position)
    {
        position = default;
        if (!world.IsValid || !body.IsValid)
        {
            return false;
        }

        Span<float> values = stackalloc float[3];
        if (Physics_GetBodyPosition(world.Value, body.Value, ref values[0]) == 0)
        {
            return false;
        }

        position = new PhysicsVec3(values[0], values[1], values[2]);
        return true;
    }

    public bool TryRayCastClosest(
        PhysicsWorldHandle world,
        PhysicsVec3 origin,
        PhysicsVec3 direction,
        float maxDistance,
        out float distance)
    {
        distance = 0.0f;
        if (!world.IsValid)
        {
            return false;
        }

        Span<float> originValues = stackalloc float[3] { origin.X, origin.Y, origin.Z };
        Span<float> directionValues = stackalloc float[3] { direction.X, direction.Y, direction.Z };
        return Physics_RayCastClosest(
            world.Value,
            ref originValues[0],
            ref directionValues[0],
            maxDistance,
            out distance) != 0;
    }

    public bool TryGetBodyRotation(
        PhysicsWorldHandle world,
        PhysicsBodyHandle body,
        out PhysicsQuat rotation)
    {
        rotation = default;
        if (!world.IsValid || !body.IsValid)
        {
            return false;
        }

        Span<float> values = stackalloc float[4];
        if (Physics_GetBodyRotation(world.Value, body.Value, ref values[0]) == 0)
        {
            return false;
        }

        rotation = new PhysicsQuat(values[0], values[1], values[2], values[3]);
        return true;
    }

    public bool TrySetBodyRotation(
        PhysicsWorldHandle world,
        PhysicsBodyHandle body,
        PhysicsQuat rotation) =>
        world.IsValid &&
        body.IsValid &&
        Physics_SetBodyRotation(world.Value, body.Value, rotation.X, rotation.Y, rotation.Z, rotation.W) != 0;

    public bool TryOverlapSphere(PhysicsWorldHandle world, PhysicsVec3 center, float radius) =>
        world.IsValid &&
        radius > 0.0f &&
        Physics_OverlapSphere(world.Value, center.X, center.Y, center.Z, radius) != 0;

    public PhysicsCharacterHandle CreateCharacter(
        PhysicsWorldHandle world,
        float radius,
        float halfHeightOfCylinder,
        PhysicsVec3 feetPosition) =>
        new(Physics_CreateCharacter(
            world.Value,
            radius,
            halfHeightOfCylinder,
            feetPosition.X,
            feetPosition.Y,
            feetPosition.Z));

    public void DestroyCharacter(PhysicsWorldHandle world, PhysicsCharacterHandle character)
    {
        if (world.IsValid && character.IsValid)
        {
            Physics_DestroyCharacter(world.Value, character.Value);
        }
    }

    public bool MoveCharacter(
        PhysicsWorldHandle world,
        PhysicsCharacterHandle character,
        float displacementX,
        float displacementZ,
        float deltaTime,
        out PhysicsVec3 feetPosition,
        out bool grounded)
    {
        feetPosition = default;
        grounded = false;
        if (!world.IsValid || !character.IsValid || deltaTime <= 0.0f)
        {
            return false;
        }

        Span<float> values = stackalloc float[3];
        int groundedFlag;
        if (Physics_MoveCharacter(
                world.Value,
                character.Value,
                displacementX,
                displacementZ,
                deltaTime,
                ref values[0],
                out groundedFlag) == 0)
        {
            return false;
        }

        feetPosition = new PhysicsVec3(values[0], values[1], values[2]);
        grounded = groundedFlag != 0;
        return true;
    }

    public bool TryGetCharacterPosition(
        PhysicsWorldHandle world,
        PhysicsCharacterHandle character,
        out PhysicsVec3 feetPosition)
    {
        feetPosition = default;
        if (!world.IsValid || !character.IsValid)
        {
            return false;
        }

        Span<float> values = stackalloc float[3];
        if (Physics_GetCharacterPosition(world.Value, character.Value, ref values[0]) == 0)
        {
            return false;
        }

        feetPosition = new PhysicsVec3(values[0], values[1], values[2]);
        return true;
    }

    public int GetCharacterCount(PhysicsWorldHandle world) =>
        world.IsValid ? Physics_GetCharacterCount(world.Value) : 0;

    public bool IsCharacterGrounded(PhysicsWorldHandle world, PhysicsCharacterHandle character) =>
        world.IsValid &&
        character.IsValid &&
        Physics_GetCharacterGroundState(world.Value, character.Value) == 1;

    [DllImport(Library, EntryPoint = "Physics_GetSolverVersion", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_GetSolverVersion(ref byte buffer, int capacity);

    [DllImport(Library, EntryPoint = "Physics_CreateWorld", CallingConvention = CallingConvention.Cdecl)]
    private static extern nint Physics_CreateWorld();

    [DllImport(Library, EntryPoint = "Physics_DestroyWorld", CallingConvention = CallingConvention.Cdecl)]
    private static extern void Physics_DestroyWorld(nint world);

    [DllImport(Library, EntryPoint = "Physics_AddStaticBox", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_AddStaticBox(
        nint world,
        float halfExtentX,
        float halfExtentY,
        float halfExtentZ,
        float positionX,
        float positionY,
        float positionZ);

    [DllImport(Library, EntryPoint = "Physics_AddCapsule", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_AddCapsule(
        nint world,
        float radius,
        float halfHeightOfCylinder,
        float positionX,
        float positionY,
        float positionZ);

    [DllImport(Library, EntryPoint = "Physics_AddDynamicBox", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_AddDynamicBox(
        nint world,
        float halfExtentX,
        float halfExtentY,
        float halfExtentZ,
        float positionX,
        float positionY,
        float positionZ,
        float friction);

    [DllImport(Library, EntryPoint = "Physics_AddStaticCapsule", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_AddStaticCapsule(
        nint world,
        float radius,
        float halfHeightOfCylinder,
        float positionX,
        float positionY,
        float positionZ);

    [DllImport(Library, EntryPoint = "Physics_Step", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_Step(nint world, int stepCount);

    [DllImport(Library, EntryPoint = "Physics_GetBodyCount", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_GetBodyCount(nint world);

    [DllImport(Library, EntryPoint = "Physics_RemoveBody", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_RemoveBody(nint world, int body);

    [DllImport(Library, EntryPoint = "Physics_GetLiveWorldCount", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_GetLiveWorldCount();

    [DllImport(Library, EntryPoint = "Physics_GetBodyPosition", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_GetBodyPosition(nint world, int body, ref float outXyz);

    [DllImport(Library, EntryPoint = "Physics_GetBodyRotation", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_GetBodyRotation(nint world, int body, ref float outXyzw);

    [DllImport(Library, EntryPoint = "Physics_SetBodyRotation", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_SetBodyRotation(
        nint world,
        int body,
        float x,
        float y,
        float z,
        float w);

    [DllImport(Library, EntryPoint = "Physics_RayCastClosest", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_RayCastClosest(
        nint world,
        ref float originXyz,
        ref float directionXyz,
        float maxDistance,
        out float outDistance);

    [DllImport(Library, EntryPoint = "Physics_OverlapSphere", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_OverlapSphere(
        nint world,
        float centerX,
        float centerY,
        float centerZ,
        float radius);

    [DllImport(Library, EntryPoint = "Physics_CreateCharacter", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_CreateCharacter(
        nint world,
        float radius,
        float halfHeightOfCylinder,
        float positionX,
        float positionY,
        float positionZ);

    [DllImport(Library, EntryPoint = "Physics_DestroyCharacter", CallingConvention = CallingConvention.Cdecl)]
    private static extern void Physics_DestroyCharacter(nint world, int character);

    [DllImport(Library, EntryPoint = "Physics_MoveCharacter", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_MoveCharacter(
        nint world,
        int character,
        float displacementX,
        float displacementZ,
        float deltaTime,
        ref float outXyz,
        out int outGrounded);

    [DllImport(Library, EntryPoint = "Physics_GetCharacterPosition", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_GetCharacterPosition(nint world, int character, ref float outXyz);

    [DllImport(Library, EntryPoint = "Physics_GetCharacterGroundState", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_GetCharacterGroundState(nint world, int character);

    [DllImport(Library, EntryPoint = "Physics_GetCharacterCount", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_GetCharacterCount(nint world);
}
