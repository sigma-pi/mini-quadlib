# Mini-QuadLib Python Bindings

Pythonic interface to the mini-quadlib C library for quadrotor control.

## Installation

### From Source (Recommended)

```bash
# Clone and build C library
git clone https://github.com/sigma-pi/mini-quadlib.git
cd mini-quadlib

# Build shared library
mkdir build && cd build
cmake .. -DBUILD_PYTHON=ON
make -j$(nproc)
cd ..

# Install Python package
cd python
pip install -e .
```

### Verify Installation

```python
import mini_quadlib as mql
print(mql.get_version())  # Should print "0.1.2"
```

### Run Basic Tests
```bash
python ./tests/test_python_api.py
```

---

## Quick Start

### Basic Usage

```python
import mini_quadlib as mql
import numpy as np

# Quaternion operations
q1 = mql.Quaternion(0.707, 0.0, 0.0, 0.707)  # 90° rotation about z
q2 = mql.Quaternion.identity()

q_normalized = q1.normalize()
q_product = q1 * q2
q_conj = q1.conjugate()

print(f"Quaternion: {q1}")
print(f"As array: {q1.to_array()}")
print(f"As dict: {q1.to_dict()}")

# Rotation matrix
R = mql.RotationMatrix.identity()
R_from_euler = mql.euler_to_rotation_matrix(np.array([0.1, 0.2, 0.3]))

# Convert between representations
q_from_R = mql.rotation_matrix_to_quaternion(R_from_euler)
euler_back = mql.rotation_matrix_to_euler(R_from_euler)

# Coordinate transforms
ned_vec = np.array([1.0, 2.0, 3.0])
enu_vec = mql.ned_to_enu(ned_vec)
ned_back = mql.enu_to_ned(enu_vec)

# Frame transforms
body_vec = mql.transform_world_to_body(ned_vec, q1)
world_vec = mql.transform_body_to_world(body_vec, q1)
```

### Geometric Controller

```python
import mini_quadlib as mql
import numpy as np

# Define current state
current_state = mql.QuadrotorState(
  position=np.array([0.0, 0.0, 0.0]),
  velocity=np.array([0.0, 0.0, 0.0]),
  attitude=mql.Quaternion.identity(),
  angular_velocity=np.array([0.0, 0.0, 0.0])
)

# Define desired setpoint
setpoint = mql.QuadrotorSetpoint(
  position=np.array([1.0, 0.0, -1.0]),  # NED frame
  velocity=np.array([0.0, 0.0, 0.0]),
  acceleration=np.array([0.0, 0.0, 0.0]),
  yaw=0.0
)

# Compute control
control = mql.geometric_control(
  current_state=current_state,
  desired_setpoint=setpoint,
  position_gains=np.array([1.0, 1.0, 2.0]),
  velocity_gains=np.array([0.5, 0.5, 1.0]),
  attitude_gains=np.array([1.0, 1.0, 1.0]),
  angular_velocity_gains=np.array([0.1, 0.1, 0.1]),
  mass=1.0,
  inertia=np.array([0.01, 0.01, 0.02])
)

print(f"Control: thrust={control[0]:.2f}, moments={control[1:4]}")
```

### L1 Adaptive Control

```python
import mini_quadlib as mql
import numpy as np

# Initialize L1 state (do this once)
l1_state = mql.L1AdaptiveState()

# Quadrotor parameters
mass = 1.0
inertia = np.array([0.01, 0.01, 0.02])
dt = 0.01  # 100 Hz

# Control loop
for i in range(100):
  # Get baseline control from geometric controller
  baseline_ctrl = mql.geometric_control(
      current_state=current_state,
      desired_setpoint=setpoint,
      position_gains=np.array([1.0, 1.0, 2.0]),
      velocity_gains=np.array([0.5, 0.5, 1.0]),
      attitude_gains=np.array([1.0, 1.0, 1.0]),
      angular_velocity_gains=np.array([0.1, 0.1, 0.1]),
      mass=mass,
      inertia=inertia
  )
  
  # Update L1 adaptive control
  l1_state = mql.l1_adaptive_control(
      previous_state=l1_state,
      current_velocity=current_state.velocity,
      current_angular_velocity=current_state.angular_velocity,
      current_attitude=current_state.attitude,
      baseline_control=baseline_ctrl,
      dt=dt,
      As_v=10.0,
      As_W=10.0,
      lpf1_cutoff_freq_force=5.0,
      lpf1_cutoff_freq_moment=5.0,
      lpf2_cutoff_freq_moment=10.0,
      mass=mass,
      inertia=inertia
  )
  
  # Get total augmented control
  total_control = l1_state.get_total_control()
  # Or: total_control = l1_state.baseline_control + l1_state.adaptive_control
  
  # Inspect disturbance estimates
  print(f"Force disturbance estimate: {l1_state.force_disturbance_estimate}")
  print(f"Moment disturbance estimate: {l1_state.moment_disturbance_estimate}")
```

---

## API Reference

### Classes

#### `Quaternion`

Represents rotation as quaternion [w, x, y, z] (scalar-first convention).

```python
# Construction
q = mql.Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
q = mql.Quaternion.identity()
q = mql.Quaternion.from_array(np.array([1, 0, 0, 0]))

# Properties
q.w, q.x, q.y, q.z  # Individual components

# Methods
q.normalize()       # Returns normalized quaternion
q.conjugate()       # Returns conjugate
q.to_array()        # Returns np.array([w, x, y, z])
q.to_dict()         # Returns {'w': ..., 'x': ..., 'y': ..., 'z': ...}

# Operators
q1 * q2             # Hamilton product (non-commutative!)
```

#### `RotationMatrix`

Represents rotation as 3x3 orthonormal matrix.

```python
# Construction
R = mql.RotationMatrix()                    # Identity
R = mql.RotationMatrix(np.eye(3))           # From numpy array
R = mql.RotationMatrix.identity()
R = mql.RotationMatrix.from_columns(colx, coly, colz)

# Properties
R.colx, R.coly, R.colz  # Column vectors (body axes in world frame)

# Methods
R.to_array()        # Returns 3x3 numpy array
R.transpose()       # Returns transposed matrix
R.inverse()         # Returns inverse (same as transpose for rotation)
```

#### `QuadrotorState`

Current state of the quadrotor.

```python
state = mql.QuadrotorState(
  position=np.array([x, y, z]),           # [m], NED frame
  velocity=np.array([vx, vy, vz]),        # [m/s], NED frame
  attitude=mql.Quaternion(...),           # Attitude quaternion
  angular_velocity=np.array([wx, wy, wz]) # [rad/s], body frame
)

state.to_dict()  # Convert to dictionary
```

#### `QuadrotorSetpoint`

Desired trajectory setpoint.

```python
setpoint = mql.QuadrotorSetpoint(
  position=np.array([x, y, z]),           # [m]
  velocity=np.array([vx, vy, vz]),        # [m/s]
  acceleration=np.array([ax, ay, az]),    # [m/s²]
  jerk=np.array([jx, jy, jz]),            # [m/s³]
  snap=np.array([sx, sy, sz]),            # [m/s⁴]
  yaw=0.0,                                # [rad]
  yaw_dot=0.0,                            # [rad/s]
  yaw_ddot=0.0                            # [rad/s²]
)
```

#### `L1AdaptiveState`

Internal state for L1 adaptive controller.

```python
l1_state = mql.L1AdaptiveState()

# After calling l1_adaptive_control():
l1_state.baseline_control           # Baseline control [f, Mx, My, Mz]
l1_state.adaptive_control           # Adaptive correction
l1_state.get_total_control()        # baseline + adaptive
l1_state.force_disturbance_estimate # Estimated force disturbance
l1_state.moment_disturbance_estimate # Estimated moment disturbance
l1_state.to_dict()                  # Full state as dictionary
```

### Functions

#### Controllers

| Function | Description |
|----------|-------------|
| `geometric_control(...)` | SE(3) geometric controller, returns `[thrust, Mx, My, Mz]` |
| `l1_adaptive_control(...)` | L1 adaptive augmentation, returns updated `L1AdaptiveState` |

#### Coordinate Transforms

| Function | Description |
|----------|-------------|
| `ned_to_enu(vec)` | Convert NED coordinates to ENU |
| `enu_to_ned(vec)` | Convert ENU coordinates to NED |
| `coordinate_transform(vec, from_frame, to_frame)` | General coordinate transform |

#### Frame Transforms

| Function | Description |
|----------|-------------|
| `transform_world_to_body(vec, attitude)` | Transform vector from world to body frame |
| `transform_body_to_world(vec, attitude)` | Transform vector from body to world frame |

#### Rotation Conversions

| Function | Description |
|----------|-------------|
| `quaternion_to_rotation_matrix(q)` | Quaternion → RotationMatrix |
| `rotation_matrix_to_quaternion(R)` | RotationMatrix → Quaternion |
| `euler_to_rotation_matrix(euler)` | Euler angles [roll, pitch, yaw] → RotationMatrix |
| `rotation_matrix_to_euler(R)` | RotationMatrix → Euler angles |

---

## Features

- **Pythonic API** - NumPy arrays, exceptions instead of error codes
- **Type Safety** - Proper input validation with helpful error messages
- **Efficient** - Thin wrapper over optimized C implementation
- **Well Tested** - Comprehensive test suite covering all functions

## Coordinate Frame

All functions use **NED (North-East-Down)** coordinate frame:
- X: North (forward)
- Y: East (right)  
- Z: Down

Quaternion convention: **scalar-first** `[w, x, y, z]`

---

## License

MIT License

## Author

Chengyu Yang (chengyuy520@gmail.com)