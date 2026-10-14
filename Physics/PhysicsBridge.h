#pragma once

// The package's native boundary. Every entry point is C, plain-data in and plain-data out, so the
// managed service binds to this header and no caller needs a solver type. Handles are opaque.

#if defined(_WIN32)
#if defined(PHYSICS_EXPORTS)
#define ARISEN_PHYSICS_API __declspec(dllexport)
#else
#define ARISEN_PHYSICS_API __declspec(dllimport)
#endif
#else
#define ARISEN_PHYSICS_API
#endif

extern "C"
{
    /// Writes the vendored solver's version into `buffer` and returns the required length. A negative
    /// return means the payload is present but the buffer is too small.
    ARISEN_PHYSICS_API int Physics_GetSolverVersion(char* buffer, int capacity);

    /// Creates a world with the default gravity, a fixed step, and one thread of its own. The solver
    /// is stepped by the caller, never from a background thread, so a replay is reproducible.
    ARISEN_PHYSICS_API void* Physics_CreateWorld();

    ARISEN_PHYSICS_API void Physics_DestroyWorld(void* world);

    /// Adds a static axis-aligned box. Returns a body handle, or -1 when the world is full.
    ARISEN_PHYSICS_API int Physics_AddStaticBox(
        void* world,
        float halfExtentX,
        float halfExtentY,
        float halfExtentZ,
        float positionX,
        float positionY,
        float positionZ);

    /// Adds a dynamic capsule, the character shape D3 builds the controller on.
    ARISEN_PHYSICS_API int Physics_AddCapsule(
        void* world,
        float radius,
        float halfHeightOfCylinder,
        float positionX,
        float positionY,
        float positionZ);

    /// Adds a dynamic box with the given friction.
    ARISEN_PHYSICS_API int Physics_AddDynamicBox(
        void* world,
        float halfExtentX,
        float halfExtentY,
        float halfExtentZ,
        float positionX,
        float positionY,
        float positionZ,
        float friction);

    /// Adds a static capsule.
    ARISEN_PHYSICS_API int Physics_AddStaticCapsule(
        void* world,
        float radius,
        float halfHeightOfCylinder,
        float positionX,
        float positionY,
        float positionZ);

    /// Advances the world by `stepCount` fixed steps. Returns 0 when the world is null.
    ARISEN_PHYSICS_API int Physics_Step(void* world, int stepCount);

    /// Bodies the world currently owns.
    ARISEN_PHYSICS_API int Physics_GetBodyCount(void* world);

    /// Removes a body the world owns. Returns 0 for an unknown handle.
    ARISEN_PHYSICS_API int Physics_RemoveBody(void* world, int body);

    /// Worlds that exist right now, so a gate can see a leak without inspecting the solver.
    ARISEN_PHYSICS_API int Physics_GetLiveWorldCount();

    /// Reads a body's position into `outXyz` (three floats). Returns 0 for an unknown body.
    ARISEN_PHYSICS_API int Physics_GetBodyPosition(void* world, int body, float* outXyz);

    /// Reads a body's rotation into `outXyzw` (four floats). Returns 0 for an unknown body.
    ARISEN_PHYSICS_API int Physics_GetBodyRotation(void* world, int body, float* outXyzw);

    /// Sets a body's rotation. The authored pose of a body is applied through this after creation.
    ARISEN_PHYSICS_API int Physics_SetBodyRotation(
        void* world,
        int body,
        float x,
        float y,
        float z,
        float w);

    /// Casts a ray and reports the closest hit distance. Returns 0 when nothing is hit.
    ARISEN_PHYSICS_API int Physics_RayCastClosest(
        void* world,
        const float* originXyz,
        const float* directionXyz,
        float maxDistance,
        float* outDistance);
}
