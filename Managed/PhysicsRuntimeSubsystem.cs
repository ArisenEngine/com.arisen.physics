using System.Numerics;
using ArisenEngine.Core.ECS;
using ArisenEngine.ECS.Lifecycle;
using ArisenKernel.Diagnostics;
using ArisenKernel.Lifecycle;

namespace ArisenEngine.Physics;

/// <summary>
/// Owns the runtime simulation: every entity that carries an authored
/// <see cref="RigidBodyComponent"/> gets a solver body, the world advances in fixed steps, and the
/// simulation writes the resulting pose back into the entity's transform so rendering never talks
/// to the solver. Bodies are created and destroyed with their entities, which is what makes the
/// simulation the owner of the body lifecycle.
/// </summary>
public sealed class PhysicsRuntimeSubsystem : ITickableSubsystem
{
    private const float FixedTimeStep = 1.0f / 60.0f;

    // A hitch may not turn into an unbounded catch-up burst; the loop keeps at most this many fixed
    // steps per frame and drops the remainder, which is a bounded-time policy on top of a
    // correctness model that never depends on how much wall time a frame took.
    private const int MaxStepsPerFrame = 8;

    private readonly Dictionary<Entity, PhysicsBodyHandle> m_Bodies = new();
    private readonly List<Entity> m_Removals = new();
    private IPhysicsService? m_Physics;
    private SceneSubsystem? m_Scenes;
    private PhysicsWorldHandle m_World;
    private EntityManager? m_BoundManager;
    private float m_Accumulator;
    private bool m_IsIdle;
    private bool m_HasLoggedStart;

    public int Priority => 90;

    public EnginePhase InitPhase => EnginePhase.Running;

    public void Initialize()
    {
#if ARISEN_ENGINE_EDITOR
        m_IsIdle = true;
        KernelLog.Info(
            "[PhysicsRuntime] Editor build detected; the editor's scene documents own authoring " +
            "previews, so the runtime simulation stays idle.");
#else
        EngineKernel kernel = EngineKernel.Instance;
        m_Scenes = kernel.GetSubsystem<SceneSubsystem>()
            ?? throw new InvalidOperationException("Physics runtime requires SceneSubsystem.");
        m_Physics = kernel.Services.GetService<IPhysicsService>();
#endif
    }

    public void Tick(float deltaTime)
    {
        if (m_IsIdle || m_Physics == null || m_Scenes == null || deltaTime <= 0.0f)
        {
            return;
        }

        EntityManager? manager = m_Scenes.ActiveEntityManager;
        if (manager == null)
        {
            ReleaseWorld();
            return;
        }

        if (!ReferenceEquals(manager, m_BoundManager))
        {
            // The scene world was replaced (a scene load, a world activation, or a sample switch),
            // so every body belongs to content that no longer exists.
            ReleaseWorld();
            m_BoundManager = manager;
        }

        ComponentPool<RigidBodyComponent> bodies = manager.GetPool<RigidBodyComponent>();
        if (!m_World.IsValid)
        {
            if (bodies.Count == 0)
            {
                // A scene with no authored bodies never pays for a solver world.
                return;
            }

            m_World = m_Physics.CreateWorld();
        }

        BindAuthoredBodies(manager, bodies);
        if (!m_HasLoggedStart && m_Bodies.Count > 0)
        {
            m_HasLoggedStart = true;
            KernelLog.InfoFormat(
                "[PhysicsRuntime] Simulation started with {0} authored body(ies).",
                m_Bodies.Count);
        }

        StepWorld(deltaTime);
        WriteBackPoses(manager, bodies);
    }

    public void Shutdown()
    {
        ReleaseWorld();
        m_BoundManager = null;
    }

    public void Dispose()
    {
        Shutdown();
    }

    private void BindAuthoredBodies(EntityManager manager, ComponentPool<RigidBodyComponent> bodies)
    {
        ReadOnlySpan<Entity> entities = bodies.GetRawEntityArray();
        for (int index = 0; index < bodies.Count; index++)
        {
            Entity entity = entities[index];
            if (m_Bodies.ContainsKey(entity))
            {
                continue;
            }

            ref RigidBodyComponent body = ref bodies.GetRef(entity);
            ref TransformComponent transform = ref manager.GetPool<TransformComponent>().GetRef(entity);
            Vector3 offset = new(body.CenterX, body.CenterY, body.CenterZ);
            Vector3 center = transform.Position + Vector3.Transform(offset, transform.Rotation);
            PhysicsVec3 position = new(center.X, center.Y, center.Z);
            PhysicsBodyHandle handle = body.Motion == PhysicsBodyMotion.Static
                ? AddStaticBody(body, position)
                : m_Physics!.AddDynamicBox(
                    m_World,
                    new PhysicsVec3(body.HalfExtentX, body.HalfExtentY, body.HalfExtentZ),
                    position,
                    body.Friction);

            if (!handle.IsValid)
            {
                KernelLog.ErrorFormat(
                    "[PhysicsRuntime] Entity '{0}' could not register its rigid body; the world is full.",
                    entity);
                continue;
            }

            if (transform.Rotation != Quaternion.Identity)
            {
                m_Physics!.TrySetBodyRotation(
                    m_World,
                    handle,
                    new PhysicsQuat(
                        transform.Rotation.X,
                        transform.Rotation.Y,
                        transform.Rotation.Z,
                        transform.Rotation.W));
            }

            m_Bodies.Add(entity, handle);
        }

        m_Removals.Clear();
        foreach (Entity tracked in m_Bodies.Keys)
        {
            if (!bodies.Has(tracked))
            {
                m_Removals.Add(tracked);
            }
        }

        for (int index = 0; index < m_Removals.Count; index++)
        {
            Entity tracked = m_Removals[index];
            m_Physics!.RemoveBody(m_World, m_Bodies[tracked]);
            m_Bodies.Remove(tracked);
        }
    }

    private PhysicsBodyHandle AddStaticBody(RigidBodyComponent body, PhysicsVec3 position) =>
        body.Shape switch
        {
            PhysicsBodyShape.Capsule => m_Physics!.AddStaticCapsule(
                m_World,
                body.Radius,
                body.HalfHeight,
                position.X,
                position.Y,
                position.Z),
            _ => m_Physics!.AddStaticBox(
                m_World,
                body.HalfExtentX,
                body.HalfExtentY,
                body.HalfExtentZ,
                position.X,
                position.Y,
                position.Z)
        };

    private void StepWorld(float deltaTime)
    {
        m_Accumulator += deltaTime;
        float maximumAccumulated = FixedTimeStep * MaxStepsPerFrame;
        if (m_Accumulator > maximumAccumulated)
        {
            m_Accumulator = maximumAccumulated;
        }

        int steps = 0;
        while (m_Accumulator >= FixedTimeStep && steps < MaxStepsPerFrame)
        {
            m_Physics!.Step(m_World, 1);
            m_Accumulator -= FixedTimeStep;
            steps++;
        }
    }

    private void WriteBackPoses(EntityManager manager, ComponentPool<RigidBodyComponent> bodies)
    {
        ComponentPool<TransformComponent> transforms = manager.GetPool<TransformComponent>();
        foreach ((Entity entity, PhysicsBodyHandle handle) in m_Bodies)
        {
            ref RigidBodyComponent authored = ref bodies.GetRef(entity);
            if (authored.Motion != PhysicsBodyMotion.Dynamic)
            {
                continue;
            }

            if (!m_Physics!.TryGetBodyPosition(m_World, handle, out PhysicsVec3 position) ||
                !m_Physics.TryGetBodyRotation(m_World, handle, out PhysicsQuat rotation))
            {
                continue;
            }

            ref TransformComponent transform = ref transforms.GetRef(entity);
            var solverRotation = new Quaternion(rotation.X, rotation.Y, rotation.Z, rotation.W);
            var solverPosition = new Vector3(position.X, position.Y, position.Z);
            Vector3 offset = new(authored.CenterX, authored.CenterY, authored.CenterZ);
            transform.Rotation = solverRotation;
            transform.Position = solverPosition - Vector3.Transform(offset, solverRotation);
        }
    }

    private void ReleaseWorld()
    {
        if (m_Physics != null && m_World.IsValid)
        {
            m_Physics.DestroyWorld(m_World);
        }

        m_World = default;
        m_Accumulator = 0.0f;
        m_Bodies.Clear();
        m_HasLoggedStart = false;
    }
}
