# Build System & Development Environment

ZenSLAM uses CMake (>=3.14) with vcpkg for dependency management.

## Configure

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      -DCMAKE_TOOLCHAIN_FILE=/path/to/vcpkg.cmake
```

## Targets

- `zenslam_core` (library)
- `zenslam_app` (GUI application)
- `hello_test` (Catch2 test executable)

## Tips

- Enable `CMAKE_VERBOSE_MAKEFILE` or use Ninja `-v` to inspect link lines.
- Use `clangd` + `compile_commands.json` symlink for rich IDE support.
- Suppress duplicate library warnings by centralizing dependencies on `zenslam_core` and linking only that into the app.
- Set `SPDLOG_LEVEL` at runtime or compile with `SPDLOG_ACTIVE_LEVEL` for performance.

## Testing

```
ctest --test-dir build -j --output-on-failure
```

Add new tests under `zenslam_tests/source/` and reference them in that CMakeLists.

## Formatting / Lint (Suggested)

- clang-format (add a `.clang-format` file)
- clang-tidy (`set(CMAKE_CXX_CLANG_TIDY ...)`)

## Continuous Integration (Future)

- Configure GitHub Actions with cache for vcpkg + build matrix.
