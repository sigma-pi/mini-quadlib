cmake_minimum_required(VERSION 3.8)
project(mini_quadlib VERSION 0.2.0 LANGUAGES C)

# Options
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(BUILD_TESTS "Build tests" OFF)
option(BUILD_PYTHON "Build Python bindings" OFF)

# Source files
set(SOURCES
  src/geometric_control.c
  src/l1_adaptive_control.c
  src/maths.c
  src/robotics.c
  src/utils.c
)

# Build library (shared for Python, static otherwise)
if(BUILD_SHARED_LIBS OR BUILD_PYTHON)
  add_library(mini_quadlib SHARED ${SOURCES})
else()
  add_library(mini_quadlib STATIC ${SOURCES})
endif()

# Include directories
target_include_directories(mini_quadlib PUBLIC include)

# Set C standard and enable GNU extensions
set_property(TARGET mini_quadlib PROPERTY C_STANDARD 99)
target_compile_definitions(mini_quadlib PRIVATE _GNU_SOURCE)

# Compiler flags
target_compile_options(mini_quadlib PRIVATE -O3 -Wall -Wextra)

# Link math library
target_link_libraries(mini_quadlib PUBLIC m)

# Build tests
if(BUILD_TESTS)
  add_executable(test_basic tests/test_basic.c)
  target_link_libraries(test_basic mini_quadlib)
endif()