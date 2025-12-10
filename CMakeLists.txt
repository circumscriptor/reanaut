cmake_minimum_required(VERSION 3.21)
project(reanaut)

find_package(Boost CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(SDL3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)

# ImGui
file(GLOB imgui_sources RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "imgui-docking/*.cpp")
add_library(imgui-docking STATIC)
target_compile_definitions(imgui-docking PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS)
target_include_directories(imgui-docking SYSTEM PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/imgui-docking")
target_link_libraries(imgui-docking PRIVATE SDL3::SDL3)
target_sources(imgui-docking PRIVATE ${imgui_sources})

# Kalman
add_library(kalman INTERFACE)
target_include_directories(kalman SYSTEM INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/kalman/include")

# PF (particle filter)
add_library(pf INTERFACE)
target_include_directories(pf SYSTEM INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/pf/include")

#
# Main target
#

add_executable(${PROJECT_NAME})

file(GLOB sources RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "source/*.cpp")
target_sources(${PROJECT_NAME} PRIVATE
    ${sources}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Boost::headers
    Eigen3::Eigen
    imgui-docking
    kalman
    opencv_highgui
    pf
    SDL3::SDL3
    # c++ c++abi
)

target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    # -stdlib=libc++
)

target_link_options(${PROJECT_NAME} PRIVATE
    # -fuse-ld=lld
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 23
    CXX_EXTENSIONS FALSE
    CXX_STANDARD_REQUIRED TRUE
    CXX_SCAN_FOR_MODULES FALSE
)
