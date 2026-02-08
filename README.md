# Mini-QuadLib

A lightweight C library for quadrotor control systems, with Python bindings.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Features

- **Geometric Controller** - SE(3) geometric control for quadrotors
- **L1 Adaptive Control** - Robust adaptive augmentation for disturbance rejection
- **Robotics Utilities** - Coordinate transforms, frame conversions, rotation representations
- **Lightweight** - Requires minimal dependencies
- **Cross-platform** - C library works anywhere, Python bindings for Linux

## Repository Structure

```
mini-quadlib/
├── include/
│   └── mini_quadlib.h      # Main C header (all API documentation here)
├── src/
│   └── *.c                 # C implementation
├── python/
│   └── mini_quadlib/       # Python package
├── build/                  # Build output (generated)
├── CMakeLists.txt
└── README.md
```

---

## C Library

### Building (Linux)

#### Prerequisites

- CMake >= 3.8
- GCC or Clang

#### Build Static Library

```bash
# Clone the repository
git clone https://github.com/sigma-pi/mini-quadlib.git
cd mini-quadlib

# Create build directory
mkdir build && cd build

# Configure and build
cmake ..
make -j$(nproc)
```

This produces:
- `build/libmini_quadlib.a` - Static library

#### Build Shared Library (for Python bindings)

```bash
mkdir build && cd build
cmake .. -DBUILD_PYTHON=ON
make -j$(nproc)
```

This produces:
- `build/libmini_quadlib.so` - Shared library

### Using the C Library

#### Include in Your Project

```c
#include "mini_quadlib.h"

int main() {
  // Get library version
  printf("mini-quadlib version: %s\n", quadlib_version());
  
  // Create state and setpoint
  state_t current_state = {
      .pos = {0.0f, 0.0f, 0.0f},
      .vel = {0.0f, 0.0f, 0.0f},
      .quat = {1.0f, 0.0f, 0.0f, 0.0f},  // w, x, y, z
      .omega = {0.0f, 0.0f, 0.0f}
  };
  
  setpoint_t desired = {
      .pos = {1.0f, 0.0f, -1.0f},  // NED frame
      .vel = {0.0f, 0.0f, 0.0f},
      .acc = {0.0f, 0.0f, 0.0f},
      .jerk = {0.0f, 0.0f, 0.0f},
      .snap = {0.0f, 0.0f, 0.0f},
      .yaw = 0.0f,
      .yaw_dot = 0.0f,
      .yaw_ddot = 0.0f
  };
  
  // Controller parameters
  geometric_params_t ctrl_params = {
      .k_p = {1.0f, 1.0f, 2.0f},
      .k_v = {0.5f, 0.5f, 1.0f},
      .k_R = {1.0f, 1.0f, 1.0f},
      .k_W = {0.1f, 0.1f, 0.1f}
  };
  
  quadx_params_t quad_params = {
      .mass = 1.0f,
      .inertia = {0.01f, 0.01f, 0.02f}
  };
  
  // Compute control
  control_4f_t output;
  quadlib_result_t result = geometric_control_fM_fullparam(
      &output, &current_state, &desired, &ctrl_params, &quad_params
  );
  
  if (result == QUADLIB_SUCCESS) {
      printf("Thrust: %.2f, Moments: [%.2f, %.2f, %.2f]\n",
             output.u1, output.u2, output.u3, output.u4);
  }
  
  return 0;
}
```

#### Compile Your Program

```bash
gcc -I/path/to/mini-quadlib/include \
  -L/path/to/mini-quadlib/build \
  your_program.c -lmini_quadlib -lm -o your_program
```

### C API Reference

All C API documentation is in the header file:

📖 **[include/mini_quadlib.h](include/mini_quadlib.h)**

Key sections:
- **Data Structures** - `vector3f_t`, `quaternion4f_t`, `state_t`, `setpoint_t`, etc.
- **Geometric Control** - `geometric_control_fM_fullparam()`
- **L1 Adaptive Control** - `l1_adaptive_control_fullparam()`
- **Coordinate Transforms** - `enu_to_ned()`, `ned_to_enu()`, `coordinate_transform_omni()`
- **Rotation Utilities** - Quaternion/Euler/RotationMatrix conversions

---

## Python Bindings

### Quick Install (from source)

```bash
cd mini-quadlib

# Build C library first
mkdir build && cd build
cmake .. -DBUILD_PYTHON=ON
make -j$(nproc)
cd ..

# Install Python package
cd python
pip install -e .
```

### Run Basic Tests
```bash
python ./python/tests/test_python_api.py
```

### Quick Test

```python
import mini_quadlib as mql
import numpy as np

print(f"Version: {mql.get_version()}")

# Create a quaternion
q = mql.Quaternion(0.707, 0.0, 0.0, 0.707)
print(f"Quaternion: {q}")

# Coordinate transform
ned = np.array([1.0, 2.0, 3.0])
enu = mql.ned_to_enu(ned)
print(f"NED {ned} -> ENU {enu}")
```

For detailed Python documentation, see:

📖 **[python/README.md](python/README.md)**

---

## License

MIT License - see [LICENSE](LICENSE) for details.

## Author

Chengyu Yang (chengyuy520@gmail.com)