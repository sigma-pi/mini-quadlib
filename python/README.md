# Mini QuadLib - Python Bindings

Python bindings for mini-quadlib: A C library for quadrotor control systems.

## Installation

```bash
pip install mini-quadlib
```

## Quick Start

```python
from mini_quadlib import Vector3, GeometricController

# Create 3D vectors
position = Vector3(0.0, 0.0, 0.0)
target_position = Vector3(1.0, 1.0, 1.0)
velocity = Vector3(0.0, 0.0, 0.0)
target_velocity = Vector3(0.0, 0.0, 0.0)

# Initialize geometric controller
controller = GeometricController(
  kp=Vector3(1.0, 1.0, 2.0),  # Position gains
  kd=Vector3(0.5, 0.5, 1.0)   # Velocity gains
)

# Compute control output
thrust, torque = controller.compute_control(
  position, velocity, target_position, target_velocity
)

print(f"Thrust command: {thrust}")
print(f"Torque command: {torque}")
```

## Features

- **3D Vector Operations**: Full 3D vector math with NumPy integration
- **Geometric Control**: Nonlinear geometric control for quadrotors
- **L1 Adaptive Control**: Adaptive control algorithms
- **Quaternion Support**: Full quaternion operations and conversions
- **Type Hints**: Complete type annotation support
- **NumPy Integration**: Seamless conversion to/from NumPy arrays

## Advanced Usage

### Working with NumPy

```python
import numpy as np
from mini_quadlib import Vector3, Matrix3x3

# Convert from NumPy
np_vec = np.array([1.0, 2.0, 3.0])
vec = Vector3.from_numpy(np_vec)

# Convert to NumPy
np_result = vec.to_numpy()

# Matrix operations
R = Matrix3x3([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
R_np = R.to_numpy()
```

### L1 Adaptive Controller

```python
from mini_quadlib import L1AdaptiveController, Vector3

controller = L1AdaptiveController(adaptation_gain=2.0)

state = Vector3(0.1, 0.2, 0.3)
reference = Vector3(1.0, 1.0, 1.0)

control_output = controller.compute_control(state, reference)
```

## Requirements

- Python >= 3.7
- NumPy >= 1.19.0
- Linux (Ubuntu, Debian, CentOS, etc.)

## License

MIT License - see LICENSE file for details.

## Contributing

Contributions welcome! Please see the main repository for development guidelines.