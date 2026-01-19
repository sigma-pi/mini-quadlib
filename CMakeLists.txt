cmake_minimum_required(VERSION 3.8)
project(mini_quadlib VERSION 1.0.0 LANGUAGES C)

# Build static library
add_library(mini_quadlib STATIC
    src/geometric_controller.c
    src/maths.c
    src/robotics.c
    src/utils.c
)

# Include directories
target_include_directories(mini_quadlib PUBLIC include)

# Set C standard
set_property(TARGET mini_quadlib PROPERTY C_STANDARD 99)

# Platform-specific definitions for math constants
if(WIN32)
    # Windows: Enable math constants like M_PI
    target_compile_definitions(mini_quadlib PUBLIC _USE_MATH_DEFINES)
elseif(UNIX)
    # Linux/Unix: Enable POSIX/GNU extensions
    target_compile_definitions(mini_quadlib PRIVATE _GNU_SOURCE)
endif()

# Compiler flags
if(MSVC)
    target_compile_options(mini_quadlib PRIVATE /W4 /O2)
else()
    target_compile_options(mini_quadlib PRIVATE -O3 -Wall -Wextra)
endif()

# # Optional: Build examples
# option(BUILD_EXAMPLES "Build examples" OFF)
# if(BUILD_EXAMPLES)
#     add_executable(simple_hover examples/simple_hover.c)
#     target_link_libraries(simple_hover mini_quadlib m)
# endif()

# Optional: Build tests
option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
    add_executable(test_basic tests/test_basic.c)
    
    # Link math library on Unix systems
    if(UNIX)
        target_link_libraries(test_basic mini_quadlib m)
    else()
        target_link_libraries(test_basic mini_quadlib)
    endif()
endif()
