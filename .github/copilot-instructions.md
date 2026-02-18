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

## Bundle Adjustment Issues & Debugging (2026-02-17)

### Problem: Excessive Pose Jumps After LBA
- Symptom: Trajectory exhibited jerky jumps (>4m pose changes) after local bundle adjustment, while normal inter-frame motion is ~0.06-0.08m
- Root cause: LBA results were applied unconditionally without validation. Optimization could converge to bad local minima due to gauge freedom in the objective function
- Key insight: Unit tests for LBA passed correctly, but integration into the full SLAM pipeline failed because landmarks from both keyframes and non-keyframes were accumulated with inconsistent poses

### Debugging Approach
- Add detailed logging of pose deltas before/after optimization: `SPDLOG_INFO("Keyframe {}: pos_delta={:.6f}m", id, pose_delta)`
- Log RMSE improvement ratio to assess optimization quality
- Compare pose changes against physical motion expectations (inter-frame motion is typically 0.06-0.1m)
- Use grep to filter relevant log messages from long output: `grep -E "(pos_delta|ACCEPTED|REJECTED)"`

### Solution: Validation Criteria for LBA Results
Applied three strict acceptance criteria before applying optimized poses:
1. **Convergence required**: `lba_result.converged == true`
2. **Minimum RMSE improvement**: At least 10% reduction in reprojection error
3. **Pose stability**: Maximum pose delta per keyframe < 0.3m (configurable threshold in slam_thread.cpp:405)

If any criterion fails, log rejection reason and discard optimization results (keeping previous poses).

### Fine-Tuning Guidance
- Thresholds in `slam_thread.cpp` line ~405: adjust RMSE ratio (0.10 = 10%) and max delta (0.3m) as needed
- High LBA rejection rate suggests poor map initialization or inconsistent landmark observations
- Monitor log output for "LBA results rejected" messages to diagnose acceptance rate
- If trajectory still jumps, reduce max pose delta threshold or increase RMSE improvement requirement