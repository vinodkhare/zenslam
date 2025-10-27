cmake_minimum_required(VERSION 3.10)

project(zenslam)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Speed-ups: ccache and Unity build
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
	message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
	set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
	set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

add_subdirectory(3rd_party/ugpm)
add_subdirectory(zenslam_app)
add_subdirectory(zenslam_core)
add_subdirectory(zenslam_options)
add_subdirectory(zenslam_tests)