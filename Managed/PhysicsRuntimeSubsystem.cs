using System.Numerics;
using ArisenEngine.Core.ECS;
using ArisenEngine.Core.Math;
using ArisenEngine.ECS.Lifecycle;
using ArisenKernel.Diagnostics;
using ArisenKernel.Contracts;
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
    private const float JumpSpeed = 5.5f;

    // A hitch may not turn into an unbounded catch-up burst; the loop keeps at most this many fixed
    // steps per frame and drops the remainder, which is a bounded-time policy on top of a
    // correctness model that never depends on how much wall time a frame took.
    private const int MaxStepsPerFrame = 8;

    private readonly Dictionary<Entity, PhysicsBodyHandle> m_Bodies = new();
    private readonly Dictionary<Entity, PhysicsCharacterHandle> m_Characters = new();
    private readonly List<Entity> m_Removals = new();
    private IPhysicsService? m_Physics;
    private SceneSubsystem? m_Scenes;
    private IInputProvider? m_Input;
    private Entity m_CameraEntity;
    private bool m_HasCamera;
    private PhysicsWorldHandle m_World;
    private EntityManager? m_BoundManager;
    private float m_Accumulator;
    private bool m_IsIdle;
    private bool m_HasLoggedStart;

    /// <summary>True while at least one authored character is bound and can be followed.</summary>
    internal bool HasViewTarget => m_Characters.Count > 0;

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
        kernel.Services.TryGetService<IInputProvider>(out m_Input);
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
        ComponentPool<CharacterControllerComponent> characters =
            manager.GetPool<CharacterControllerComponent>();
        if (!m_World.IsValid)
        {
            if (bodies.Count == 0 && characters.Count == 0)
            {
                // A scene with no authored bodies or characters never pays for a solver world.
                return;
            }

            m_World = m_Physics.CreateWorld();
        }

        BindAuthoredBodies(manager, bodies);
        BindCharacters(manager, characters);
        if (!m_HasLoggedStart && (m_Bodies.Count > 0 || m_Characters.Count > 0))
        {
            m_HasLoggedStart = true;
            KernelLog.InfoFormat(
                "[PhysicsRuntime] Simulation started with {0} authored body(ies) and {1} character(s).",
                m_Bodies.Count,
                m_Characters.Count);
        }

        StepWorld(deltaTime);
        MoveCharacters(deltaTime, manager, characters);
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

    private void BindCharacters(
        EntityManager manager,
        ComponentPool<CharacterControllerComponent> characters)
    {
        ReadOnlySpan<Entity> entities = characters.GetRawEntityArray();
        ComponentPool<TransformComponent> transforms = manager.GetPool<TransformComponent>();
        for (int index = 0; index < characters.Count; index++)
        {
            Entity entity = entities[index];
            if (m_Characters.ContainsKey(entity))
            {
                continue;
            }

            ref CharacterControllerComponent controller = ref characters.GetRef(entity);
            ref TransformComponent transform = ref transforms.GetRef(entity);
            // The character's base is the entity's transform origin, so the visible capsule mesh
            // lines up with the collision capsule without an offset.
            PhysicsCharacterHandle handle = m_Physics!.CreateCharacter(
                m_World,
                controller.Radius,
                controller.HalfHeight,
                new PhysicsVec3(transform.Position.X, transform.Position.Y, transform.Position.Z));
            if (!handle.IsValid)
            {
                KernelLog.ErrorFormat(
                    "[PhysicsRuntime] Entity '{0}' could not register its character.",
                    entity);
                continue;
            }

            m_Characters.Add(entity, handle);
        }

        m_Removals.Clear();
        foreach (Entity tracked in m_Characters.Keys)
        {
            if (!characters.Has(tracked))
            {
                m_Removals.Add(tracked);
            }
        }

        for (int index = 0; index < m_Removals.Count; index++)
        {
            Entity tracked = m_Removals[index];
            m_Physics!.DestroyCharacter(m_World, m_Characters[tracked]);
            m_Characters.Remove(tracked);
        }
    }

    private void MoveCharacters(
        float deltaTime,
        EntityManager manager,
        ComponentPool<CharacterControllerComponent> characters)
    {
        if (m_Characters.Count == 0)
        {
            return;
        }

        ComponentPool<TransformComponent> transforms = manager.GetPool<TransformComponent>();
        foreach ((Entity entity, PhysicsCharacterHandle handle) in m_Characters)
        {
            ref CharacterControllerComponent controller = ref characters.GetRef(entity);
            Vector2 input = ReadMoveInput();
            float speed = controller.MoveSpeed;
            float displacementX = 0.0f;
            float displacementZ = 0.0f;
            if (m_Input != null && m_Input.IsKeyDown(InputKey.Space))
            {
                m_Physics!.TryJumpCharacter(m_World, handle, JumpSpeed);
            }

            if (input.LengthSquared() > 1e-6f && speed > 0.0f)
            {
                Vector3 direction = ToWorldDirection(input);
                float length = MathF.Sqrt(input.LengthSquared());
                displacementX = direction.X / length * speed * deltaTime;
                displacementZ = direction.Z / length * speed * deltaTime;
            }

            if (!m_Physics!.MoveCharacter(
                    m_World,
                    handle,
                    displacementX,
                    displacementZ,
                    deltaTime,
                    out PhysicsVec3 feet,
                    out bool _))
            {
                continue;
            }

            ref TransformComponent transform = ref transforms.GetRef(entity);
            transform.Position = new Vector3(feet.X, feet.Y, feet.Z);
        }
    }

    /// <summary>
    /// The authored character's state for gates: its feet height and whether the solver reports it
    /// on walkable ground.
    /// </summary>
    internal bool TryGetCharacterObservation(Entity entity, out float feetY, out bool grounded)
    {
        feetY = 0.0f;
        grounded = false;
        if (m_Physics == null || !m_Characters.TryGetValue(entity, out PhysicsCharacterHandle handle))
        {
            return false;
        }

        if (!m_Physics.TryGetCharacterPosition(m_World, handle, out PhysicsVec3 feet))
        {
            return false;
        }

        feetY = feet.Y;
        grounded = m_Physics.IsCharacterGrounded(m_World, handle);
        return true;
    }

    /// <summary>
    /// The point an interactive viewer should follow: the first authored character's chest. The
    /// shell's camera consumes this through <c>IRuntimeViewTarget</c>, so it never has to know
    /// which package owns the player.
    /// </summary>
    internal bool TryGetViewTarget(out Vector3 worldPosition)
    {
        worldPosition = default;
        EntityManager? manager = m_Scenes?.ActiveEntityManager;
        if (m_Physics == null || manager == null || m_Characters.Count == 0)
        {
            return false;
        }

        ComponentPool<CharacterControllerComponent> controllers =
            manager.GetPool<CharacterControllerComponent>();
        foreach ((Entity entity, PhysicsCharacterHandle handle) in m_Characters)
        {
            if (!controllers.Has(entity) ||
                !m_Physics.TryGetCharacterPosition(m_World, handle, out PhysicsVec3 feet))
            {
                continue;
            }

            ref CharacterControllerComponent controller = ref controllers.GetRef(entity);
            worldPosition = new Vector3(
                feet.X,
                feet.Y + controller.HalfHeight + controller.Radius,
                feet.Z);
            return true;
        }

        return false;
    }

    private Vector2 ReadMoveInput()
    {
        if (m_Input == null)
        {
            return Vector2.Zero;
        }

        float forward = 0.0f;
        if (m_Input.IsKeyDown(InputKey.W)) forward += 1.0f;
        if (m_Input.IsKeyDown(InputKey.S)) forward -= 1.0f;
        float right = 0.0f;
        if (m_Input.IsKeyDown(InputKey.D)) right += 1.0f;
        if (m_Input.IsKeyDown(InputKey.A)) right -= 1.0f;
        return new Vector2(right, forward);
    }

    private Vector3 ToWorldDirection(Vector2 input)
    {
        EntityManager? manager = m_Scenes?.ActiveEntityManager;
        if (manager != null && TryResolveCameraRotation(manager, out Quaternion rotation))
        {
            Vector3 forward = Vector3.Transform(MathExtensions.Forward, rotation);
            forward.Y = 0.0f;
            Vector3 right = rotation.RightVector();
            right.Y = 0.0f;
            Vector3 direction = forward * input.Y + right * input.X;
            if (direction.LengthSquared() > 1e-6f)
            {
                return Vector3.Normalize(direction);
            }
        }

        return new Vector3(input.X, 0.0f, input.Y);
    }

    private bool TryResolveCameraRotation(EntityManager manager, out Quaternion rotation)
    {
        ComponentPool<CameraComponent> cameras = manager.GetPool<CameraComponent>();
        ComponentPool<TransformComponent> transforms = manager.GetPool<TransformComponent>();
        if (m_HasCamera &&
            manager.IsAlive(m_CameraEntity) &&
            cameras.Has(m_CameraEntity) &&
            transforms.Has(m_CameraEntity))
        {
            rotation = transforms.GetRef(m_CameraEntity).Rotation;
            return true;
        }

        m_HasCamera = false;
        ReadOnlySpan<Entity> entities = cameras.GetRawEntityArray();
        for (int index = 0; index < cameras.Count; index++)
        {
            Entity candidate = entities[index];
            if (!manager.IsAlive(candidate) || !transforms.Has(candidate))
            {
                continue;
            }

            m_CameraEntity = candidate;
            m_HasCamera = true;
            rotation = transforms.GetRef(candidate).Rotation;
            return true;
        }

        rotation = Quaternion.Identity;
        return false;
    }

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
        m_Characters.Clear();
        m_HasCamera = false;
        m_HasLoggedStart = false;
    }
}
