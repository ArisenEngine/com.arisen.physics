# com.arisen.physics

Rigid-body simulation behind a package boundary (roadmap Track D). Gameplay packages depend on
`ArisenEngine.Physics.IPhysicsService`; Jolt, its bodies, and its shapes never leave this package.

- `3rdparty/jolt` - the vendored Jolt solver (D18), refreshed by copying an upstream tag's `Jolt/`
  directory; `3rdparty/jolt/README.md` records the tag and the procedure.
- `Physics` - the package payload: a flat C ABI (`PhysicsBridge.h`) over the solver. `Physics.dll` is
  the package-owned native runtime the workspace deploys.
- `Managed` - the boundary contract, its binding to the payload, and the package entry that registers
  the service.

D1 delivered this boundary: a fixed-step world, static boxes, character capsules, body positions, and
a closest-hit ray query, with the solver initialized per process and stepped only by the caller. D2
adds cooked collision from authored meshes and body ownership by the simulation; D3 adds the character
controller and the hit/hurt queries combat reads.