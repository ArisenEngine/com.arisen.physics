using System.Numerics;
using ArisenKernel.Contracts;
using ArisenKernel.Lifecycle;

namespace ArisenEngine.Physics;

/// <summary>
/// Publishes the character the interactive viewer should follow. The shell's camera resolves
/// <see cref="IRuntimeViewTarget"/> and never learns which package owns the player, so this is the
/// only place the two sides meet.
/// </summary>
internal sealed class CharacterViewTarget : IRuntimeViewTarget
{
    private PhysicsRuntimeSubsystem? m_Subsystem;

    public bool IsActive => Resolve()?.HasViewTarget == true;

    public bool TryGetTarget(out Vector3 worldPosition)
    {
        PhysicsRuntimeSubsystem? subsystem = Resolve();
        if (subsystem != null)
        {
            return subsystem.TryGetViewTarget(out worldPosition);
        }

        worldPosition = default;
        return false;
    }

    private PhysicsRuntimeSubsystem? Resolve() =>
        m_Subsystem ??= EngineKernel.Instance.GetSubsystem<PhysicsRuntimeSubsystem>();
}
