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

# gd-imgui-cocos' main branch still uses the old CCIMEDelegate/CCKeyboardDelegate
# signatures; current Geode added extra parameters. Patch them in place (idempotent).
set(_IC_DIR "${${CPM_LAST_PACKAGE_NAME}_SOURCE_DIR}")
foreach(_f "${_IC_DIR}/include/imgui-cocos.hpp" "${_IC_DIR}/src/main.cpp")
    if(EXISTS "${_f}")
        file(READ "${_f}" _txt)
        string(REGEX REPLACE "keyDown\\(([A-Za-z0-9_:]*enumKeyCodes) ([A-Za-z0-9_]+)\\)" "keyDown(\\1 \\2, double)" _new "${_txt}")
        string(REGEX REPLACE "keyUp\\(([A-Za-z0-9_:]*enumKeyCodes) ([A-Za-z0-9_]+)\\)" "keyUp(\\1 \\2, double)" _new "${_new}")
        string(REGEX REPLACE "insertText\\(const char ?\\* ?([A-Za-z0-9_]+), ?int ([A-Za-z0-9_]+)\\)" "insertText(const char* \\1, int \\2, cocos2d::enumKeyCodes)" _new "${_new}")
        if(NOT "${_new}" STREQUAL "${_txt}")
            file(WRITE "${_f}" "${_new}")
            message(STATUS "Patched ${_f} for new Geode signatures")
        endif()
    endif()
endforeach()

target_link_libraries(${PROJECT_NAME} imgui-cocos)

# GDR2 replay writer (works in Eclipse, xdBot, ...)
CPMAddPackage("gh:maxnut/GDReplayFormat#gdr2")
target_link_libraries(${PROJECT_NAME} libGDR)

setup_geode_mod(${PROJECT_NAME})
