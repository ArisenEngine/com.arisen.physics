using ArisenKernel.Diagnostics;
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

    public void OnLoad(IServiceRegistry services)
    {
        m_Service = new PhysicsService();
        services.RegisterService<IPhysicsService>(m_Service);
        KernelLog.InfoFormat(
            "[PhysicsPackage] Registered physics service on solver {0}.",
            m_Service.SolverVersion);
    }

    public void OnUnload(IServiceRegistry services)
    {
        m_Service = null;
        KernelLog.Info("[PhysicsPackage] Unloaded physics service.");
    }
}
