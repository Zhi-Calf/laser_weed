# Clipper2 (vendored, C++)

Upstream: https://github.com/AngusJohnson/Clipper2
Tag: `Clipper2_1.5.4` (tarball `Clipper2_1.5.4.tar.gz` from
`https://codeload.github.com/AngusJohnson/Clipper2/tar.gz/refs/tags/Clipper2_1.5.4`)
License: Boost Software License 1.0 (see `LICENSE`, © Angus Johnson)
Taken from: `CPP/Clipper2Lib/` (headers + the three translation units).

## What is here

```
include/clipper2/*.h    clipper.h, clipper.core.h, clipper.engine.h, clipper.offset.h,
                        clipper.rectclip.h, clipper.minkowski.h, clipper.version.h
src/*.cpp               clipper.engine.cpp, clipper.offset.cpp, clipper.rectclip.cpp
```

`clipper.export.h` (the C API) and `CPP/Utils/`, `CPP/Examples/`, `CPP/Tests/` are
NOT vendored -- nothing here needs them.

## Why vendored instead of a system dependency

Jammy/Humble has no `libclipper2-dev` (universe only ships the legacy
`libpolyclipping-dev`), and the tractor is built offline: a `FetchContent`/apt step
inside the build would turn "colcon build" into something that needs the network.
The sources are small (~7.7k lines, three .cpp), permissively licensed, and pinned
to a release tag, so a copy in-tree is the cheapest reproducible option.

`global_path_planner/CMakeLists.txt` builds them into a static library `clipper2`
and links it into the planner node. The vendored code is compiled with warnings
disabled on purpose (`-w`): it is not ours to fix, and the package's
`-Wall -Wextra -Wpedantic` (+ the `cppcheck` test, which only scans the package
directory) must keep reporting on our own sources only.

## Updating

Replace `include/clipper2/` and `src/` with the same two directories from a newer
tag, keep the bundled `LICENSE`, and re-run the package tests (the bow-planner
unit tests exercise the offsetting/clipping this library does for us).
