# Copilot Instructions

## Build Instructions

- Build folder is `.build` in the parent directory.
- Build folder contains subfolders for each build type, e.g., `Debug`, `Release`, 'RelWithDebInfo' etc.
- Use `vcpkg` toolchain file for dependency management.
- When building, try to use VS Code/CMake build tasks if possible.
- The github repo is located here: https://github.com/vinodkhare/zenslam

## Session Learnings (2026-02-17)

- Always build and run the application in `Release` mode unless the user explicitly requests another configuration.
- Default build command: `cmake --build .build/Release -j`.
- Default run command: `./.build/Release/zenslam_app/zenslam_app --options-file zenslam_options/options/tumvi.yaml`.
- Running the app with no CLI args may show help instead of starting the SLAM pipeline; pass an explicit options file when validating runtime behavior.
- For long-running app runs, use bounded timeouts and inspect captured output files to avoid terminal interleaving/noise.