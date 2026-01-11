# mini-quadlib

A lightweight C library for quadrotor control, providing essential functions for flying quadrotors.

## Features

- **Geometric Controller**: Non-linear geometric controller for quadrotor trajectory tracking
- **Coordinate Conversions**: Seamless conversion between quaternions, Euler angles, and rotation matrices
- **Frame Conversions**: Support for multiple coordinate frames (ENU, NED, NWU, FLU, FRD)
- **Vector/Matrix Operations**: Comprehensive 3D vector and rotation matrix operations
- **Lightweight**: Single header file + implementation, written in pure C99
- **No Dependencies**: Only requires standard math library

## Building

### Build the library and example:
```bash
make
```

### Run the example program:
```bash
make run
```

### Clean build artifacts:
```bash
make clean
```

## Quick Start

```c
#include "mini_quadlib.h"

int main(void) {
    // Create a quadrotor state
    QuadrotorState state;
    state.position = vector3_create(0.0, 0.0, 1.0);
    state.velocity = vector3_create(0.0, 0.0, 0.0);
    state.attitude = quaternion_create(1.0, 0.0, 0.0, 0.0);
    state.angular_vel = vector3_create(0.0, 0.0, 0.0);
    
    // Set desired position
    Vector3 desired_pos = vector3_create(1.0, 1.0, 2.0);
    Vector3 desired_vel = vector3_create(0.0, 0.0, 0.0);
    Vector3 desired_acc = vector3_create(0.0, 0.0, 0.0);
    double desired_yaw = 0.0;
    
    // Get controller gains
    GeometricControllerGains gains = geometric_controller_default_gains();
    
    // Compute control command
    ControlCommand cmd = geometric_controller_compute(
        state, desired_pos, desired_vel, desired_acc, desired_yaw, gains
    );
    
    // Use cmd.thrust and cmd.moment to control the quadrotor
    return 0;
}
```

## API Reference

### Coordinate Conversions

Convert between different attitude representations:

```c
// Quaternion ↔ Euler angles
Quaternion q = euler_to_quaternion(euler);
EulerAngles e = quaternion_to_euler(quat);

// Quaternion ↔ Rotation matrix
RotationMatrix R = quaternion_to_rotation_matrix(quat);
Quaternion q = rotation_matrix_to_quaternion(R);

// Euler angles ↔ Rotation matrix
RotationMatrix R = euler_to_rotation_matrix(euler);
EulerAngles e = rotation_matrix_to_euler(R);
```

### Frame Conversions

Convert vectors between different coordinate frames:

```c
// ENU to NED
Vector3 v_ned = convert_frame(v_enu, FRAME_ENU, FRAME_NED);

// NED to ENU
Vector3 v_enu = convert_frame(v_ned, FRAME_NED, FRAME_ENU);

// Supported frames: FRAME_ENU, FRAME_NED, FRAME_NWU, FRAME_FLU, FRAME_FRD
```

### Geometric Controller

Compute control commands for trajectory tracking:

```c
GeometricControllerGains gains = geometric_controller_default_gains();
gains.mass = 1.5;  // Customize parameters
gains.kx = 6.0;    // Position gain
gains.kv = 4.0;    // Velocity gain

ControlCommand cmd = geometric_controller_compute(
    state, desired_pos, desired_vel, desired_acc, desired_yaw, gains
);
```

### Vector Operations

```c
Vector3 v1 = vector3_create(1.0, 2.0, 3.0);
Vector3 v2 = vector3_create(4.0, 5.0, 6.0);

Vector3 sum = vector3_add(v1, v2);
Vector3 diff = vector3_subtract(v1, v2);
double dot = vector3_dot(v1, v2);
Vector3 cross = vector3_cross(v1, v2);
double norm = vector3_norm(v1);
Vector3 normalized = vector3_normalize(v1);
```

## File Structure

```
mini-quadlib/
├── mini_quadlib.h      # Main header file with all declarations
├── mini_quadlib.c      # Implementation of all functions
├── example.c           # Example usage program
├── Makefile            # Build system
└── README.md           # This file
```

## Coordinate Frames

The library supports the following coordinate frame conventions:

- **ENU (East-North-Up)**: Common in geodesy and some robotics applications
  - X: East, Y: North, Z: Up
- **NED (North-East-Down)**: Common in aerospace and aviation
  - X: North, Y: East, Z: Down
- **NWU (North-West-Up)**: Alternative navigation frame
  - X: North, Y: West, Z: Up
- **FLU (Forward-Left-Up)**: Body-fixed frame
  - X: Forward, Y: Left, Z: Up
- **FRD (Forward-Right-Down)**: Body-fixed frame
  - X: Forward, Y: Right, Z: Down

## Geometric Controller

The geometric controller is based on the paper:
"Geometric tracking control of a quadrotor UAV on SE(3)" by Lee et al.

Key features:
- Operates on the Special Euclidean group SE(3)
- Provides exponential stability guarantees
- No singularities (unlike Euler angle-based controllers)
- Suitable for aggressive maneuvers

Controller gains can be tuned via `GeometricControllerGains`:
- `kx`: Position gain (controls position tracking stiffness)
- `kv`: Velocity gain (controls damping)
- `kR`: Attitude gain (controls attitude tracking stiffness)
- `kOmega`: Angular velocity gain (controls rotational damping)

## Future Additions

Planned features for future releases:
- L1 adaptive control augmentation
- Model Predictive Control (MPC)
- Additional controller types
- More coordinate frame support

## License

See LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.
