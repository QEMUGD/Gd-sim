cmake_minimum_required(VERSION 3.21)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_OSX_DEPLOYMENT_TARGET "10.13" CACHE STRING "" FORCE)
set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64" CACHE STRING "" FORCE)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

project(Pathfinder VERSION 1.0.0)

add_library(${PROJECT_NAME} SHARED src/main.cpp)

if (NOT DEFINED ENV{GEODE_SDK})
    message(FATAL_ERROR "Unable to find Geode SDK! Please define GEODE_SDK.")
endif()
add_subdirectory($ENV{GEODE_SDK} ${CMAKE_CURRENT_BINARY_DIR}/geode)

# ImGui for Cocos2d (Windows + Android)
CPMAddPackage("gh:matcool/gd-imgui-cocos#main")
target_link_libraries(${PROJECT_NAME} imgui-cocos)

# GDR2 replay writer (works in Eclipse, xdBot, ...)
CPMAddPackage("gh:maxnut/GDReplayFormat#gdr2")
target_link_libraries(${PROJECT_NAME} libGDR)

setup_geode_mod(${PROJECT_NAME})
