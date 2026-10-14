using ArisenEngine.Resources.Serialization;
using ArisenKernel.Diagnostics;
using ArisenKernel.Lifecycle;
using ArisenKernel.Packages;
using ArisenKernel.Services;

namespace ArisenEngine.Physics;

/// <summary>
/// Registers the simulation behind <see cref="IPhysicsService"/>. A profile that does not mount this
/// package has no physics, which is what keeps the dependency explicit for every product.
/// </summary>
public sealed class PhysicsPackage : IPackageEntry
{
    public const string PackageId = "com.arisen.physics";

    private PhysicsService? m_Service;
    private RigidBodySceneComponentCodec? m_SceneComponentCodec;
    private ISceneComponentExtensionRegistry? m_SceneComponentRegistry;
    private IRuntimeSmokeScenarioRegistry? m_SmokeScenarioRegistry;
    private PhysicsSettleSmokeScenarioProvider? m_SmokeScenarioProvider;

    public void OnLoad(IServiceRegistry services)
    {
        m_Service = new PhysicsService();
        services.RegisterService<IPhysicsService>(m_Service);
        m_SceneComponentRegistry = services.GetService<ISceneComponentExtensionRegistry>();
        m_SceneComponentCodec = new RigidBodySceneComponentCodec();
        m_SceneComponentRegistry.Register(m_SceneComponentCodec);
        m_SmokeScenarioRegistry = services.GetService<IRuntimeSmokeScenarioRegistry>();
        m_SmokeScenarioProvider = new PhysicsSettleSmokeScenarioProvider(m_Service);
        m_SmokeScenarioRegistry.Register(
            PhysicsSettleSmokeScenarioProvider.ModeName,
            m_SmokeScenarioProvider);
        KernelLog.InfoFormat(
            "[PhysicsPackage] Registered physics service on solver {0} and the authored rigid body component.",
            m_Service.SolverVersion);
    }

    public void OnUnload(IServiceRegistry services)
    {
        if (m_SmokeScenarioRegistry != null && m_SmokeScenarioProvider != null)
        {
            m_SmokeScenarioRegistry.Unregister(
                PhysicsSettleSmokeScenarioProvider.ModeName,
                m_SmokeScenarioProvider);
        }

        m_SmokeScenarioRegistry = null;
        m_SmokeScenarioProvider = null;
        if (m_SceneComponentRegistry != null && m_SceneComponentCodec != null)
        {
            m_SceneComponentRegistry.Unregister(m_SceneComponentCodec);
        }

        m_SceneComponentRegistry = null;
        m_SceneComponentCodec = null;
        m_Service = null;
        KernelLog.Info("[PhysicsPackage] Unloaded physics service.");
    }
}
