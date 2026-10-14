# com.arisen.physics

Rigid-body simulation behind a package boundary (roadmap Track D). Gameplay packages depend on
`ArisenEngine.Physics.IPhysicsService`; Jolt, its bodies, and its shapes never leave this package.

- `3rdparty/jolt` - the vendored Jolt solver (D18), refreshed by copying an upstream tag's `Jolt/`
  directory; `3rdparty/jolt/README.md` records the tag and the procedure.
- `Physics` - the package payload: a flat C ABI (`PhysicsBridge.h`) over the solver. `Physics.dll` is
  the package-owned native runtime the workspace deploys.
- `Managed` - the boundary contract, its binding to the payload, the `RigidBody` scene component the
  package registers with the scene extension registry, the runtime subsystem that binds entities to
  bodies and steps the world, and the package entry that registers the service and its bounded
  `physics-settle` smoke scenario.

D1 delivered this boundary: a fixed-step world, static boxes, character capsules, body positions, and
a closest-hit ray query, with the solver initialized per process and stepped only by the caller.
D2's first slice adds authored rigid bodies: explicit box/capsule shapes with a center offset are
declared on scene entities, the runtime creates and destroys bodies with their entities, steps the
world in fixed increments, and writes the pose back into the entity transform. What remains of D2 is
cooked collision derived from the authored meshes; D3 adds the character controller and the hit/hurt
queries combat reads.
