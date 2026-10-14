# Vendored Jolt

Upstream: https://github.com/jrouwe/JoltPhysics
Tag: v5.6.0 (commit e77f175595e64cb44218cc9d9d56fc365ad0e36a)
Licensed under the MIT license in `LICENSE`; the upstream README and its contribution agreement are
not vendored because the library directory is self-contained.

`Jolt/` is the upstream library directory, copied unchanged. The package's CMake includes
`Jolt/Jolt.cmake` with `PHYSICS_REPO_ROOT` pointing at this directory, which keeps the upstream
include convention (`<Jolt/...>`) and the upstream source list intact.

Refresh procedure: check out the wanted tag, copy its `Jolt/` directory over this one, update the
tag and commit above. The D18 decision records why the solver is vendored instead of fetched: no
network access during a build.