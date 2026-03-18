cmake_minimum_required(VERSION 3.16)

project(zenslam_metal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)

find_package(OpenCV CONFIG REQUIRED)
message(STATUS "Found OpenCV: " ${OpenCV_VERSION})

if(APPLE)
    enable_language(OBJCXX)
endif()

add_library(${PROJECT_NAME}
    source/pyr_lk.cpp
    source/pyr_lk_factory.cpp
)

if(APPLE)
    target_sources(${PROJECT_NAME} PRIVATE source/pyr_lk_metal.mm)
    target_compile_definitions(${PROJECT_NAME} PRIVATE ZENSLAM_METAL_ENABLED=1)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${CMAKE_SOURCE_DIR}/zenslam_core/include
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    opencv_core
    opencv_video
)

if(APPLE)
    target_link_libraries(${PROJECT_NAME} PUBLIC
        "-framework Foundation"
        "-framework Metal"
    )
endif()
