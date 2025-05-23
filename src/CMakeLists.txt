cmake_minimum_required(VERSION 3.13)



###############################################################################
## LIBRARY-PROJECT
## name and version
###############################################################################
project(MspParser VERSION 1.0.0 LANGUAGES CXX)



###############################################################################
## SETTINGS
## basic project settings before use
###############################################################################
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Enabling export of all symbols to create a dynamic library
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# creating output directory architecture in accordance with GNU guidelines
set(BINARY_DIR "${CMAKE_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${BINARY_DIR}/lib")
file (GLOB_RECURSE IN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.h.in)
configure_file(${IN_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Version.h)



###############################################################################
## TARGET
## create target and add include path
###############################################################################
# create glob files for *.h, *.cpp
file (GLOB_RECURSE H_FILES   ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file (GLOB_RECURSE CPP_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file (GLOB_RECURSE C_FILES   ${CMAKE_CURRENT_SOURCE_DIR}/*.c)

# concatenate the results (glob files) to variable
set  (SOURCES ${CPP_FILES} ${H_FILES} ${C_FILES})
# create lib from src
if (NOT TARGET ${PROJECT_NAME})
    add_library(${PROJECT_NAME} ${SOURCES})
endif()
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})



###############################################################################
## LINK LIBRARIES
## linking all dependencies
###############################################################################