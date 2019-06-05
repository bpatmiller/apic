# project defn
cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME apic)
project(${PROJECT_NAME})

# cmake options
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# cpp version / settings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-fopenmp -Wall -Wextra -Werror -fmax-errors=1 -Wno-unused-parameter -Wno-unused-variable")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# source / lib directories
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
aux_source_directory(${SRC_DIR} SOURCES)
add_executable(${PROJECT_NAME} ${SOURCES})

# GLM
FIND_PACKAGE(glm 0.9.8.0 QUIET)
INCLUDE_DIRECTORIES(${glm_INCLUDE_DIR})
add_definitions(-DGLM_ENABLE_EXPERIMENTAL -DGLM_FORCE_SIZE_FUNC=1 -DGLM_FORCE_RADIANS=1)

# partio
include_directories(lib/partio)
link_directories(lib/partio)
