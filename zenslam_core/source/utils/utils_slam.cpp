// utils_slam.cpp - Legacy unified implementation file
//
// This file has been refactored into focused modules for better maintainability:
//  - correspondence_utils.cpp: All correspondence extraction functions
//  - rigid_transform.cpp: Rigid body transformation estimation (estimate_rigid, umeyama, etc.)
//  - matching_utils.cpp: Feature matching and PnP (filter, create_matcher, match_*, solve_pnp)
//  - triangulation_utils.cpp: 3D reconstruction (triangulate_points, triangulate_keylines)
//  - tracking_utils.cpp: Optical flow tracking (track_keylines)
//
// The utils_slam.h header now includes all these modules, maintaining backward compatibility.
// All implementations have been moved to their respective specialized modules.

#include "zenslam/utils/utils_slam.h"

// This file intentionally left minimal - all implementations are in specialized modules
