# Copilot Instructions

## Tips for Building, Running and Debugging

- Build folder is `.build` in the parent directory.
- Build folder contains subfolders for each build type, e.g., `Debug`, `Release`, `RelWithDebInfo` etc.
- When building, use CMake and don't call ninja directly
- Always build and run the application in `Release` mode unless the user explicitly requests another configuration.
- Default configure command: `/opt/local/bin/cmake -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -DCMAKE_C_COMPILER:FILEPATH=/usr/bin/clang -DCMAKE_CXX_COMPILER:FILEPATH=/usr/bin/clang++ -Wno-dev -DCMAKE_TOOLCHAIN_FILE=/Volumes/DATA01/source/github.com/microsoft/vcpkg/scripts/buildsystems/vcpkg.cmake --no-warn-unused-cli -S /Volumes/DATA01/source/github.com/vinodkhare/zenslam -B /Volumes/DATA01/source/github.com/vinodkhare/zenslam/.build/Release -G Ninja`
- Default build command: `cmake --build .build/Release -j`.
- Default run command: `./.build/Release/zenslam_app/zenslam_app --options-file zenslam_options/options/tumvi.yaml`.
- Running the app with no CLI args may show help instead of starting the SLAM pipeline; pass an explicit options file when validating runtime behavior.
- For long-running app runs, use bounded timeouts and inspect captured output files to avoid terminal interleaving/noise.
- To debug the application consider adding logging statements and examining the output to understand the issue
- When looking at build output, don't use `tail`, instead always look at all output

## Tips for Code Generation
- Use the RAII (Resource Acquisition Is Initialization) pattern for managing resources in C++ to ensure proper cleanup and avoid memory leaks.
- Optimize for simplicity and ease of use, both for the user and the developer
- Optimize for long term maintenance and extensibility. 
