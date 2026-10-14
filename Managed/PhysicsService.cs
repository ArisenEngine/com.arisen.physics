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

    public bool Step(PhysicsWorldHandle world, int stepCount) =>
        world.IsValid && Physics_Step(world.Value, stepCount) != 0;

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

    [DllImport(Library, EntryPoint = "Physics_Step", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_Step(nint world, int stepCount);

    [DllImport(Library, EntryPoint = "Physics_GetBodyPosition", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_GetBodyPosition(nint world, int body, ref float outXyz);

    [DllImport(Library, EntryPoint = "Physics_RayCastClosest", CallingConvention = CallingConvention.Cdecl)]
    private static extern int Physics_RayCastClosest(
        nint world,
        ref float originXyz,
        ref float directionXyz,
        float maxDistance,
        out float outDistance);
}
