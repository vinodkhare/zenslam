cmake_minimum_required(VERSION 3.14)

project(zenslam_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)

find_package(Boost CONFIG REQUIRED COMPONENTS program_options)
message(STATUS "Found Boost: " ${Boost_VERSION})

find_package(Microsoft.GSL CONFIG REQUIRED)
message(STATUS "Found Microsoft.GSL: " ${Microsoft.GSL_VERSION})

find_package(OpenCV CONFIG REQUIRED)
message(STATUS "Found OpenCV: " ${OpenCV_VERSION})

# Native VTK rendering (replace OpenCV viz for 3D scene)
find_package(VTK COMPONENTS
    CommonCore
    CommonDataModel
    FiltersCore
    InteractionStyle
    RenderingCore
    RenderingOpenGL2
    OPTIONAL_COMPONENTS CommonColor
    REQUIRED)
message(STATUS "Found VTK: ${VTK_VERSION}")

find_package(concurrentqueue CONFIG REQUIRED)
message(STATUS "Found concurrentqueue: " ${concurrentqueue_VERSION})

find_package(hello-imgui CONFIG REQUIRED)
message(STATUS "Found hello-imgui: " ${hello-imgui_VERSION})

find_package(magic_enum CONFIG REQUIRED)
message(STATUS "Found magic_enum: " ${magic_enum_VERSION})

find_package(spdlog CONFIG REQUIRED)
message(STATUS "Found spdlog: " ${spdlog_VERSION})

find_package(yaml-cpp CONFIG REQUIRED)
message(STATUS "Found yaml-cpp: " ${yaml-cpp_VERSION})

hello_imgui_add_app(zenslam_app source/main.cpp
        source/application.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} PRIVATE
        concurrentqueue::concurrentqueue
        magic_enum::magic_enum
        opencv_highgui
        opencv_calib3d
        opencv_core
        opencv_features2d
        opencv_imgproc
        # Removed opencv_viz in favor of native VTK rendering
        spdlog::spdlog
        yaml-cpp::yaml-cpp
        zenslam_core
        Boost::program_options
        Microsoft.GSL::GSL
        VTK::CommonCore
        VTK::CommonDataModel
        VTK::FiltersCore
        VTK::InteractionStyle
        VTK::RenderingCore
        VTK::RenderingOpenGL2
)

# VTK module auto-init for correct factory registration
vtk_module_autoinit(
  TARGETS ${PROJECT_NAME}
  MODULES VTK::CommonCore VTK::CommonDataModel VTK::FiltersCore VTK::InteractionStyle VTK::RenderingCore VTK::RenderingOpenGL2
)
target_compile_definitions(${PROJECT_NAME} PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
