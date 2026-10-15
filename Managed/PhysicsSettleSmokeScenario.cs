using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using ArisenEngine.Core.ECS;
using ArisenEngine.ECS.Lifecycle;
using ArisenKernel.Diagnostics;
using ArisenKernel.Lifecycle;

namespace ArisenEngine.Physics;

internal sealed class PhysicsSettleSmokeScenarioProvider : IRuntimeSmokeScenarioProvider
{
    public const string ModeName = "physics-settle";

    private readonly IPhysicsService m_Physics;

    public PhysicsSettleSmokeScenarioProvider(IPhysicsService physics)
    {
        m_Physics = physics;
    }

    public bool TryCreateScenario(
        RuntimeSmokeScenarioContext context,
        out IRuntimeSmokeScenario scenario,
        out string diagnostic)
    {
        if (!string.Equals(context.ModeName, ModeName, StringComparison.Ordinal))
        {
            scenario = null!;
            diagnostic = $"Physics does not provide smoke scenario '{context.ModeName}'.";
            return false;
        }

        scenario = new PhysicsSettleSmokeScenario(context, m_Physics);
        diagnostic = string.Empty;
        return true;
    }
}

/// <summary>
/// The D2 wiring gate: the profile boots its scene, and this scenario requires every authored
/// dynamic body to fall from where the scene placed it, come to rest, and stay at rest, then
/// requires the shutdown to leave no solver world behind. It observes the entity transforms the
/// simulation writes back, so a pass means the whole path - authored component, solver body,
/// fixed stepping, transform write-back - works, not only the solver.
/// </summary>
internal sealed class PhysicsSettleSmokeScenario : IRuntimeSmokeScenario
{
    private const int RequiredRestFrames = 30;
    private const float MinimumFallDistance = 0.5f;
    private const float RestThreshold = 0.001f;

    private readonly RuntimeSmokeScenarioContext m_Context;
    private readonly IPhysicsService m_Physics;
    private readonly Dictionary<Entity, BodyObservation> m_Bodies = new();
    private readonly Dictionary<Entity, CharacterObservation> m_Characters = new();
    private PhysicsRuntimeSubsystem? m_Runtime;
    private SceneSubsystem? m_Scenes;
    private string? m_FailureMessage;
    private bool m_ReadyForShutdown;
    private bool m_Complete;
    private int m_LiveWorldsAfterShutdown = -1;
    private uint m_LastFrame;

    public PhysicsSettleSmokeScenario(RuntimeSmokeScenarioContext context, IPhysicsService physics)
    {
        m_Context = context;
        m_Physics = physics;
        OutputPath = string.IsNullOrWhiteSpace(context.OutputPath)
            ? GetDefaultOutputPath(context.WorkspacePath, context.ProfileName)
            : Path.GetFullPath(context.OutputPath);
    }

    public string Name => PhysicsSettleSmokeScenarioProvider.ModeName;
    public string OutputPath { get; }
    public bool IsReadyForShutdown => m_ReadyForShutdown;
    public bool IsComplete => m_Complete;
    public bool Succeeded => m_Complete && m_FailureMessage == null;
    public string? FailureMessage => m_FailureMessage;

    public void Start(uint initialFrameIndex)
    {
        m_LastFrame = initialFrameIndex;
        m_Scenes = EngineKernel.Instance.GetSubsystem<SceneSubsystem>();
        m_Runtime = EngineKernel.Instance.GetSubsystem<PhysicsRuntimeSubsystem>();
        if (m_Scenes == null)
        {
            ReportFailure("The scene subsystem is not available for the physics settle gate.");
        }
    }

    public void BeforeFrame(uint frameIndex)
    {
    }

    public void AfterFrame(uint frameIndex)
    {
        m_LastFrame = frameIndex;
        if (m_ReadyForShutdown || m_Scenes == null)
        {
            return;
        }

        EntityManager? manager = m_Scenes.ActiveEntityManager;
        if (manager == null)
        {
            return;
        }

        ComponentPool<RigidBodyComponent> authored = manager.GetPool<RigidBodyComponent>();
        ComponentPool<TransformComponent> transforms = manager.GetPool<TransformComponent>();
        ReadOnlySpan<Entity> entities = authored.GetRawEntityArray();
        for (int index = 0; index < authored.Count; index++)
        {
            Entity entity = entities[index];
            ref RigidBodyComponent body = ref authored.GetRef(entity);
            if (body.Motion != PhysicsBodyMotion.Dynamic)
            {
                continue;
            }

            ref TransformComponent transform = ref transforms.GetRef(entity);
            if (!m_Bodies.TryGetValue(entity, out BodyObservation? observation))
            {
                observation = new BodyObservation(transform.Position.Y);
                m_Bodies.Add(entity, observation);
                KernelLog.InfoFormat(
                    "[PhysicsSettle] Observing dynamic body at Y {0:F2}.",
                    transform.Position.Y);
            }

            if (Math.Abs(transform.Position.Y - observation.PreviousY) < RestThreshold)
            {
                observation.RestFrames++;
            }
            else
            {
                observation.RestFrames = 0;
            }

            observation.PreviousY = transform.Position.Y;
            observation.SettledY = transform.Position.Y;
        }

        // Characters authored in the scene must be standing on walkable ground by the time the
        // gate completes; the character controller is otherwise only exercised by its own gate.
        ComponentPool<CharacterControllerComponent> controllers =
            manager.GetPool<CharacterControllerComponent>();
        ReadOnlySpan<Entity> controllerEntities = controllers.GetRawEntityArray();
        for (int index = 0; index < controllers.Count; index++)
        {
            Entity entity = controllerEntities[index];
            if (!m_Characters.TryGetValue(entity, out CharacterObservation? character))
            {
                character = new CharacterObservation();
                m_Characters.Add(entity, character);
            }

            if (m_Runtime != null &&
                m_Runtime.TryGetCharacterObservation(entity, out float feetY, out bool grounded))
            {
                character.FeetY = feetY;
                character.Grounded = grounded;
            }
        }

        if (m_Bodies.Count == 0)
        {
            return;
        }

        foreach (BodyObservation observation in m_Bodies.Values)
        {
            if (!observation.IsSettled)
            {
                return;
            }
        }

        foreach (CharacterObservation character in m_Characters.Values)
        {
            if (!character.Grounded)
            {
                return;
            }
        }

        foreach (BodyObservation observation in m_Bodies.Values)
        {
            if (observation.SettledY > observation.StartY - MinimumFallDistance)
            {
                ReportFailure(
                    $"A dynamic body only moved from Y {observation.StartY:F3} to " +
                    $"{observation.SettledY:F3}; the gate requires it to fall at least " +
                    $"{MinimumFallDistance:F2} and settle.");
                return;
            }
        }

        KernelLog.InfoFormat(
            "[PhysicsSettle] {0} dynamic body(ies) settled and {1} character(s) grounded after {2} frame(s).",
            m_Bodies.Count,
            m_Characters.Count,
            frameIndex);
        m_ReadyForShutdown = true;
    }

    public void ReportFailure(string message)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(message);
        m_FailureMessage ??= message;
        m_ReadyForShutdown = true;
    }

    public void AfterShutdown()
    {
        m_LiveWorldsAfterShutdown = m_Physics.LiveWorldCount;
        if (m_LiveWorldsAfterShutdown != 0)
        {
            m_FailureMessage ??=
                $"Shutdown left {m_LiveWorldsAfterShutdown} physics world(s) alive.";
        }

        m_Complete = true;
        WriteSummary();
    }

    private void WriteSummary()
    {
        var artifact = new PhysicsSettleSmokeArtifact(
            SchemaVersion: 1,
            Mode: Name,
            Profile: m_Context.ProfileName,
            Workspace: Path.GetFullPath(m_Context.WorkspacePath),
            Passed: Succeeded,
            Frames: m_LastFrame,
            LivePhysicsWorldsAfterShutdown: m_LiveWorldsAfterShutdown,
            Bodies: m_Bodies.Values
                .Select(observation => new PhysicsSettleBodyArtifact(
                    observation.StartY,
                    observation.SettledY,
                    observation.RestFrames,
                    observation.IsSettled))
                .ToArray(),
            Characters: m_Characters.Values
                .Select(observation => new PhysicsSettleCharacterArtifact(
                    observation.FeetY,
                    observation.Grounded))
                .ToArray(),
            Failure: m_FailureMessage);

        string? directory = Path.GetDirectoryName(OutputPath);
        if (!string.IsNullOrWhiteSpace(directory))
        {
            Directory.CreateDirectory(directory);
        }

        string temporaryPath = OutputPath + ".tmp." + Guid.NewGuid().ToString("N");
        string json = JsonSerializer.Serialize(artifact, new JsonSerializerOptions
        {
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            WriteIndented = true,
            Converters = { new JsonStringEnumConverter() }
        });
        File.WriteAllText(temporaryPath, json, new UTF8Encoding(false));
        File.Move(temporaryPath, OutputPath, overwrite: true);
    }

    private static string GetDefaultOutputPath(string workspacePath, string profileName)
    {
        string safeProfile = string.Concat(profileName.Select(character =>
            Path.GetInvalidFileNameChars().Contains(character) ? '_' : character));
        return Path.GetFullPath(Path.Combine(
            workspacePath,
            ".arisen",
            "Logs",
            $"physics-settle-summary-{safeProfile}-latest.json"));
    }

    private sealed class BodyObservation
    {
        public BodyObservation(float startY)
        {
            StartY = startY;
            PreviousY = startY;
            SettledY = startY;
        }

        public float StartY { get; }
        public float PreviousY { get; set; }
        public float SettledY { get; set; }
        public int RestFrames { get; set; }
        public bool IsSettled => RestFrames >= RequiredRestFrames;
    }

    private sealed class CharacterObservation
    {
        public float FeetY { get; set; }
        public bool Grounded { get; set; }
    }

    private sealed record PhysicsSettleSmokeArtifact(
        int SchemaVersion,
        string Mode,
        string Profile,
        string Workspace,
        bool Passed,
        uint Frames,
        int LivePhysicsWorldsAfterShutdown,
        IReadOnlyList<PhysicsSettleBodyArtifact> Bodies,
        IReadOnlyList<PhysicsSettleCharacterArtifact> Characters,
        string? Failure);

    private sealed record PhysicsSettleBodyArtifact(
        float StartY,
        float SettledY,
        int RestFrames,
        bool Settled);

    private sealed record PhysicsSettleCharacterArtifact(
        float FeetY,
        bool Grounded);
}
