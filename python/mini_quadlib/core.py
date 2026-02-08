"""
Mini-QuadLib Python API - Pythonic function-based interface
Following the design principles: simple, function-based, closer to C API
"""

import ctypes
import os
from typing import List, Tuple, Optional, Union, Dict, Any
import numpy as np

# =============================================================================
# LIBRARY LOADING
# =============================================================================

def _find_library():
    """Locate the compiled C library"""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    lib_path = os.path.join(current_dir, '..', '..', 'build', 'libmini_quadlib.so')
    
    if os.path.exists(lib_path):
        return lib_path
    
    raise FileNotFoundError(
        "Could not find libmini_quadlib.so. "
        "Please build it first with: ./build_python.sh"
    )

# Load the library
try:
    _lib_path = _find_library()
    _lib = ctypes.CDLL(_lib_path)
except Exception as e:
    raise ImportError(f"Failed to load mini_quadlib library: {e}")

# =============================================================================
# INTERNAL C STRUCTURES (NOT EXPOSED TO PYTHON API)
# =============================================================================

class _vector3f_t(ctypes.Structure):
    """Internal C vector structure"""
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]

class _matrix3f_t(ctypes.Structure):
    """Internal C matrix structure"""
    _fields_ = [("colx", _vector3f_t), ("coly", _vector3f_t), ("colz", _vector3f_t)]

class _quaternion4f_t(ctypes.Structure):
    """Internal C quaternion structure"""
    _fields_ = [("w", ctypes.c_float), ("x", ctypes.c_float), ("y", ctypes.c_float), ("z", ctypes.c_float)]

class _control_4f_t(ctypes.Structure):
    """Internal C control structure"""
    _fields_ = [("u1", ctypes.c_float), ("u2", ctypes.c_float), ("u3", ctypes.c_float), ("u4", ctypes.c_float)]

class _state_t(ctypes.Structure):
    """Internal C state structure"""
    _fields_ = [
        ("pos", _vector3f_t), ("vel", _vector3f_t), 
        ("quat", _quaternion4f_t), ("omega", _vector3f_t)
    ]

class _l1_state_t(ctypes.Structure):
    """Internal C L1 state structure"""
    _fields_ = [
        ("vel", _vector3f_t), ("omega", _vector3f_t), ("vel_hat", _vector3f_t), ("omega_hat", _vector3f_t),
        ("sigma_f_hat", _vector3f_t), ("sigma_M_hat", _vector3f_t), ("quat", _quaternion4f_t),
        ("ub", _control_4f_t), ("uad", _control_4f_t), ("lpf1", _control_4f_t), ("lpf2", _control_4f_t)
    ]

class _setpoint_t(ctypes.Structure):
    """Internal C setpoint structure"""
    _fields_ = [
        ("pos", _vector3f_t), ("vel", _vector3f_t), ("acc", _vector3f_t), 
        ("jerk", _vector3f_t), ("snap", _vector3f_t),
        ("yaw", ctypes.c_float), ("yaw_dot", ctypes.c_float), ("yaw_ddot", ctypes.c_float)
    ]

class _geometric_params_t(ctypes.Structure):
    """Internal C geometric params structure"""
    _fields_ = [("k_p", _vector3f_t), ("k_v", _vector3f_t), ("k_R", _vector3f_t), ("k_W", _vector3f_t)]

class _l1_params_t(ctypes.Structure):
    """Internal C L1 params structure"""
    _fields_ = [
        ("As_v", ctypes.c_float), ("As_W", ctypes.c_float), 
        ("lpf1_cutoffreq_f", ctypes.c_float), ("lpf1_cutoffreq_M", ctypes.c_float), ("lpf2_cutoffreq_M", ctypes.c_float)
    ]

class _quadx_params_t(ctypes.Structure):
    """Internal C quadrotor params structure"""
    _fields_ = [
        ("inertia", _vector3f_t), ("mass", ctypes.c_float), ("xlen", ctypes.c_float), 
        ("ylen", ctypes.c_float), ("k_eta", ctypes.c_float), ("k_m", ctypes.c_float)
    ]

# =============================================================================
# UTILITY FUNCTIONS FOR CONVERSION
# =============================================================================

def _numpy_to_vector3f(arr: np.ndarray) -> _vector3f_t:
    """Convert numpy array to internal C vector"""
    arr = np.asarray(arr, dtype=np.float32).flatten()
    if len(arr) != 3:
        raise ValueError("Vector must have exactly 3 elements")
    return _vector3f_t(float(arr[0]), float(arr[1]), float(arr[2]))

def _vector3f_to_numpy(vec: _vector3f_t) -> np.ndarray:
    """Convert internal C vector to numpy array"""
    return np.array([vec.x, vec.y, vec.z], dtype=np.float32)

def _numpy_to_matrix3f(arr: np.ndarray) -> _matrix3f_t:
    """Convert numpy array to internal C matrix (column-major)"""
    arr = np.asarray(arr, dtype=np.float32)
    if arr.shape != (3, 3):
        raise ValueError("Matrix must be 3x3")
    mat = _matrix3f_t()
    # Convert row-major numpy to column-major C structure
    mat.colx = _numpy_to_vector3f(arr[:, 0])
    mat.coly = _numpy_to_vector3f(arr[:, 1]) 
    mat.colz = _numpy_to_vector3f(arr[:, 2])
    return mat

def _matrix3f_to_numpy(mat: _matrix3f_t) -> np.ndarray:
    """Convert internal C matrix to numpy array (row-major)"""
    return np.column_stack([
        _vector3f_to_numpy(mat.colx),
        _vector3f_to_numpy(mat.coly),
        _vector3f_to_numpy(mat.colz)
    ])

def _control4f_to_numpy(ctrl: _control_4f_t) -> np.ndarray:
    """Convert internal C control to numpy array"""
    return np.array([ctrl.u1, ctrl.u2, ctrl.u3, ctrl.u4], dtype=np.float32)

def _numpy_to_control4f(arr: np.ndarray) -> _control_4f_t:
    """Convert numpy array to internal C control"""
    arr = np.asarray(arr, dtype=np.float32).flatten()
    if len(arr) != 4:
        raise ValueError("Control must have exactly 4 elements")
    return _control_4f_t(float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3]))

# =============================================================================
# FUNCTION SETUP AND ERROR HANDLING
# =============================================================================

def _setup_c_functions():
    """Setup C function signatures"""
    functions = {}
    
    # Try to setup each function, skip if not available
    def try_setup(name, argtypes, restype):
        try:
            func = getattr(_lib, name)
            func.argtypes = argtypes
            func.restype = restype
            functions[name] = func
            return True
        except AttributeError:
            return False
    
    # Version function
    try_setup('quadlib_version', [], ctypes.c_char_p)
    
    # Core controller functions
    try_setup('geometric_control_fM_fullparam', [
        ctypes.POINTER(_control_4f_t), ctypes.POINTER(_state_t), ctypes.POINTER(_setpoint_t),
        ctypes.POINTER(_geometric_params_t), ctypes.POINTER(_quadx_params_t)
    ], ctypes.c_int)
    
    try_setup('l1_adaptive_control_fullparam', [
        ctypes.POINTER(_l1_state_t), ctypes.POINTER(_l1_state_t), ctypes.c_float,
        ctypes.POINTER(_l1_params_t), ctypes.POINTER(_quadx_params_t)
    ], ctypes.c_int)
    
    # Robotics tools
    try_setup('frame_world_to_body', [
        ctypes.POINTER(_vector3f_t), ctypes.POINTER(_vector3f_t), ctypes.POINTER(_quaternion4f_t)
    ], ctypes.c_int)
    
    try_setup('frame_body_to_world', [
        ctypes.POINTER(_vector3f_t), ctypes.POINTER(_vector3f_t), ctypes.POINTER(_quaternion4f_t)
    ], ctypes.c_int)
    
    try_setup('coordinate_transform_omni', [
        ctypes.POINTER(_vector3f_t), ctypes.c_char_p, ctypes.POINTER(_vector3f_t), ctypes.c_char_p
    ], ctypes.c_int)
    
    try_setup('enu_to_ned', [ctypes.POINTER(_vector3f_t), ctypes.POINTER(_vector3f_t)], ctypes.c_int)
    try_setup('ned_to_enu', [ctypes.POINTER(_vector3f_t), ctypes.POINTER(_vector3f_t)], ctypes.c_int)
    
    # Quaternion operations
    try_setup('quaternion_normalize', [ctypes.POINTER(_quaternion4f_t), ctypes.POINTER(_quaternion4f_t)], ctypes.c_int)
    try_setup('quaternion_multiply', [ctypes.POINTER(_quaternion4f_t), ctypes.POINTER(_quaternion4f_t), ctypes.POINTER(_quaternion4f_t)], ctypes.c_int)
    try_setup('quaternion_conjugate', [ctypes.POINTER(_quaternion4f_t), ctypes.POINTER(_quaternion4f_t)], ctypes.c_int)
    
    # Rotation conversions
    try_setup('rotation_matrix_to_quaternion', [ctypes.POINTER(_quaternion4f_t), ctypes.POINTER(_matrix3f_t)], ctypes.c_int)
    try_setup('quaternion_to_rotation_matrix', [ctypes.POINTER(_matrix3f_t), ctypes.POINTER(_quaternion4f_t)], ctypes.c_int)
    try_setup('euler_angles_to_rotation_matrix', [ctypes.POINTER(_matrix3f_t), ctypes.POINTER(_vector3f_t)], ctypes.c_int)
    try_setup('rotation_matrix_to_euler_angles', [ctypes.POINTER(_vector3f_t), ctypes.POINTER(_matrix3f_t)], ctypes.c_int)
    
    return functions

_c_functions = _setup_c_functions()

def _check_result(result: int, function_name: str):
    """Check C function result and raise exception if error"""
    if result == 0:  # SUCCESS
        return
    elif result == -1:  # WARNING
        print(f"Warning in {function_name}")
        return
    elif result == -2:
        raise ValueError(f"Invalid input number in {function_name}")
    elif result == -3:
        raise ValueError(f"Invalid input string in {function_name}")
    elif result == -4:
        raise RuntimeError(f"Safety violation in {function_name}")
    else:
        raise RuntimeError(f"Unknown error {result} in {function_name}")

def _call_c_function(func_name: str, *args):
    """Call C function with error checking"""
    if func_name not in _c_functions:
        raise NotImplementedError(f"Function {func_name} not available in C library")
    result = _c_functions[func_name](*args)
    _check_result(result, func_name)

# =============================================================================
# SPECIAL PYTHON CLASSES FOR COMPLEX DATA TYPES
# =============================================================================

class Quaternion:
    """
    Quaternion class for representing rotations
    Stores quaternion as [w, x, y, z] (scalar-first convention)
    """
    
    def __init__(self, w: float = 1.0, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        """Initialize quaternion with w, x, y, z components"""
        self._data = np.array([w, x, y, z], dtype=np.float32)
    
    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'Quaternion':
        """Create quaternion from numpy array [w, x, y, z]"""
        arr = np.asarray(arr, dtype=np.float32).flatten()
        if len(arr) != 4:
            raise ValueError("Quaternion array must have exactly 4 elements")
        return cls(arr[0], arr[1], arr[2], arr[3])
    
    @classmethod
    def identity(cls) -> 'Quaternion':
        """Create identity quaternion"""
        return cls(1.0, 0.0, 0.0, 0.0)
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array [w, x, y, z]"""
        return self._data.copy()
    
    def to_dict(self) -> Dict[str, float]:
        """Convert to dictionary"""
        return {'w': self.w, 'x': self.x, 'y': self.y, 'z': self.z}
    
    @property
    def w(self) -> float: return float(self._data[0])
    @property 
    def x(self) -> float: return float(self._data[1])
    @property
    def y(self) -> float: return float(self._data[2])
    @property
    def z(self) -> float: return float(self._data[3])
    
    def _to_c_struct(self) -> _quaternion4f_t:
        """Convert to internal C structure"""
        return _quaternion4f_t(self.w, self.x, self.y, self.z)
    
    @classmethod
    def _from_c_struct(cls, c_quat: _quaternion4f_t) -> 'Quaternion':
        """Create from internal C structure"""
        return cls(c_quat.w, c_quat.x, c_quat.y, c_quat.z)
    
    def normalize(self) -> 'Quaternion':
        """Return a normalized (unit-length) copy of this quaternion"""
        c_input = self._to_c_struct()
        c_output = _quaternion4f_t()
        _call_c_function('quaternion_normalize', ctypes.byref(c_output), ctypes.byref(c_input))
        return Quaternion._from_c_struct(c_output)

    def multiply(self, other: 'Quaternion') -> 'Quaternion':
        """Multiply this quaternion by another (Hamilton product, NOT commutative)"""
        c_q1 = self._to_c_struct()
        c_q2 = other._to_c_struct()
        c_output = _quaternion4f_t()
        _call_c_function('quaternion_multiply', ctypes.byref(c_output), ctypes.byref(c_q1), ctypes.byref(c_q2))
        return Quaternion._from_c_struct(c_output)

    def conjugate(self) -> 'Quaternion':
        """Return the conjugate of this quaternion"""
        c_input = self._to_c_struct()
        c_output = _quaternion4f_t()
        _call_c_function('quaternion_conjugate', ctypes.byref(c_output), ctypes.byref(c_input))
        return Quaternion._from_c_struct(c_output)

    def __mul__(self, other: 'Quaternion') -> 'Quaternion':
        """Operator overload for quaternion multiplication: q1 * q2"""
        return self.multiply(other)

    def __repr__(self):
        return f"Quaternion(w={self.w:.3f}, x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f})"

class RotationMatrix:
    """
    Rotation matrix class for 3D rotations
    Stores as column vectors (colx, coly, colz) representing body frame axes in world frame
    """
    
    def __init__(self, matrix: Optional[np.ndarray] = None):
        """Initialize rotation matrix from 3x3 numpy array or identity"""
        if matrix is None:
            self._data = np.eye(3, dtype=np.float32)
        else:
            matrix = np.asarray(matrix, dtype=np.float32)
            if matrix.shape != (3, 3):
                raise ValueError("Rotation matrix must be 3x3")
            self._data = matrix.copy()
    
    @classmethod
    def identity(cls) -> 'RotationMatrix':
        """Create identity rotation matrix"""
        return cls()
    
    @classmethod
    def from_columns(cls, colx: np.ndarray, coly: np.ndarray, colz: np.ndarray) -> 'RotationMatrix':
        """Create rotation matrix from column vectors"""
        matrix = np.column_stack([colx, coly, colz])
        return cls(matrix)
    
    def to_array(self) -> np.ndarray:
        """Convert to 3x3 numpy array"""
        return self._data.copy()
    
    @property
    def colx(self) -> np.ndarray:
        """First column (x-axis of body frame in world frame)"""
        return self._data[:, 0].copy()
    
    @property
    def coly(self) -> np.ndarray:
        """Second column (y-axis of body frame in world frame)"""
        return self._data[:, 1].copy()
    
    @property
    def colz(self) -> np.ndarray:
        """Third column (z-axis of body frame in world frame)"""
        return self._data[:, 2].copy()
    
    def transpose(self) -> 'RotationMatrix':
        """Return transposed rotation matrix"""
        return RotationMatrix(self._data.T)
    
    def inverse(self) -> 'RotationMatrix':
        """Return inverse rotation matrix (same as transpose for rotation matrices)"""
        return self.transpose()
    
    def _to_c_struct(self) -> _matrix3f_t:
        """Convert to internal C structure"""
        return _numpy_to_matrix3f(self._data)
    
    @classmethod
    def _from_c_struct(cls, c_mat: _matrix3f_t) -> 'RotationMatrix':
        """Create from internal C structure"""
        return cls(_matrix3f_to_numpy(c_mat))
    
    def __repr__(self):
        return f"RotationMatrix(\n{self._data}\n)"

class QuadrotorState:
    """
    Quadrotor state containing position, velocity, attitude, and angular velocity
    """
    
    def __init__(self, position: np.ndarray = None, velocity: np.ndarray = None, 
                attitude: Quaternion = None, angular_velocity: np.ndarray = None):
        """Initialize quadrotor state"""
        self.position = np.zeros(3, dtype=np.float32) if position is None else np.asarray(position, dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32) if velocity is None else np.asarray(velocity, dtype=np.float32)
        self.attitude = Quaternion.identity() if attitude is None else attitude
        self.angular_velocity = np.zeros(3, dtype=np.float32) if angular_velocity is None else np.asarray(angular_velocity, dtype=np.float32)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        return {
            'position': self.position.tolist(),
            'velocity': self.velocity.tolist(),
            'attitude': self.attitude.to_dict(),
            'angular_velocity': self.angular_velocity.tolist()
        }
    
    def _to_c_struct(self) -> _state_t:
        """Convert to internal C structure"""
        state = _state_t()
        state.pos = _numpy_to_vector3f(self.position)
        state.vel = _numpy_to_vector3f(self.velocity)
        state.quat = self.attitude._to_c_struct()
        state.omega = _numpy_to_vector3f(self.angular_velocity)
        return state

class QuadrotorSetpoint:
    """
    Desired setpoint for quadrotor including position, velocity, acceleration, jerk, snap, and yaw
    """
    
    def __init__(self, position: np.ndarray = None, velocity: np.ndarray = None, 
                acceleration: np.ndarray = None, jerk: np.ndarray = None, snap: np.ndarray = None,
                yaw: float = 0.0, yaw_dot: float = 0.0, yaw_ddot: float = 0.0):
        """Initialize quadrotor setpoint"""
        self.position = np.zeros(3, dtype=np.float32) if position is None else np.asarray(position, dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32) if velocity is None else np.asarray(velocity, dtype=np.float32)
        self.acceleration = np.zeros(3, dtype=np.float32) if acceleration is None else np.asarray(acceleration, dtype=np.float32)
        self.jerk = np.zeros(3, dtype=np.float32) if jerk is None else np.asarray(jerk, dtype=np.float32)
        self.snap = np.zeros(3, dtype=np.float32) if snap is None else np.asarray(snap, dtype=np.float32)
        self.yaw = float(yaw)
        self.yaw_dot = float(yaw_dot)
        self.yaw_ddot = float(yaw_ddot)
    
    def _to_c_struct(self) -> _setpoint_t:
        """Convert to internal C structure"""
        setpoint = _setpoint_t()
        setpoint.pos = _numpy_to_vector3f(self.position)
        setpoint.vel = _numpy_to_vector3f(self.velocity)
        setpoint.acc = _numpy_to_vector3f(self.acceleration)
        setpoint.jerk = _numpy_to_vector3f(self.jerk)
        setpoint.snap = _numpy_to_vector3f(self.snap)
        setpoint.yaw = self.yaw
        setpoint.yaw_dot = self.yaw_dot
        setpoint.yaw_ddot = self.yaw_ddot
        return setpoint

# =============================================================================
# VERSION FUNCTION
# =============================================================================

def get_version() -> str:
    """Get the version string of the mini-quadlib library"""
    if 'quadlib_version' in _c_functions:
        try:
            version_bytes = _c_functions['quadlib_version']()
            return version_bytes.decode('utf-8')
        except Exception:
            pass
    return "0.1.2"  # fallback

# =============================================================================
# CORE CONTROLLER FUNCTIONS
# =============================================================================

def geometric_control(current_state: QuadrotorState, 
                      desired_setpoint: QuadrotorSetpoint,
                      position_gains: np.ndarray,
                      velocity_gains: np.ndarray, 
                      attitude_gains: np.ndarray,
                      angular_velocity_gains: np.ndarray,
                      quadrotor_mass: float,
                      quadrotor_inertia: np.ndarray,
                      arm_length_x: float,
                      arm_length_y: float,
                      thrust_coefficient: float,
                      moment_coefficient: float) -> np.ndarray:
    """
    Geometric control for quadrotor
    
    Returns:
        Control output as numpy array [thrust, moment_x, moment_y, moment_z]
    """
    # Create parameter structures
    ctrl_params = _geometric_params_t()
    ctrl_params.k_p = _numpy_to_vector3f(position_gains)
    ctrl_params.k_v = _numpy_to_vector3f(velocity_gains)
    ctrl_params.k_R = _numpy_to_vector3f(attitude_gains)
    ctrl_params.k_W = _numpy_to_vector3f(angular_velocity_gains)
    
    quad_params = _quadx_params_t()
    quad_params.mass = float(quadrotor_mass)
    quad_params.inertia = _numpy_to_vector3f(quadrotor_inertia)
    quad_params.xlen = float(arm_length_x)
    quad_params.ylen = float(arm_length_y)
    quad_params.k_eta = float(thrust_coefficient)
    quad_params.k_m = float(moment_coefficient)
    
    # Convert states
    c_current_state = current_state._to_c_struct()
    c_desired_setpoint = desired_setpoint._to_c_struct()
    
    # Call C function
    output_control = _control_4f_t()
    _call_c_function('geometric_control_fM_fullparam',
                    ctypes.byref(output_control),
                    ctypes.byref(c_current_state),
                    ctypes.byref(c_desired_setpoint),
                    ctypes.byref(ctrl_params),
                    ctypes.byref(quad_params))
    
    return _control4f_to_numpy(output_control)

# =============================================================================
# ROBOTICS UTILITY FUNCTIONS
# =============================================================================

def transform_world_to_body(world_vector: np.ndarray, attitude: Quaternion) -> np.ndarray:
    """Transform vector from world frame to body frame"""
    c_world_vec = _numpy_to_vector3f(world_vector)
    c_attitude = attitude._to_c_struct()
    c_body_vec = _vector3f_t()
    
    _call_c_function('frame_world_to_body',
                    ctypes.byref(c_body_vec),
                    ctypes.byref(c_world_vec),
                    ctypes.byref(c_attitude))
    
    return _vector3f_to_numpy(c_body_vec)

def transform_body_to_world(body_vector: np.ndarray, attitude: Quaternion) -> np.ndarray:
    """Transform vector from body frame to world frame"""
    c_body_vec = _numpy_to_vector3f(body_vector)
    c_attitude = attitude._to_c_struct()
    c_world_vec = _vector3f_t()
    
    _call_c_function('frame_body_to_world',
                    ctypes.byref(c_world_vec),
                    ctypes.byref(c_body_vec),
                    ctypes.byref(c_attitude))
    
    return _vector3f_to_numpy(c_world_vec)

def coordinate_transform(vector: np.ndarray, from_frame: str, to_frame: str) -> np.ndarray:
    """Transform coordinate vector from one frame to another (e.g., NED to ENU)"""
    c_input_vec = _numpy_to_vector3f(vector)
    c_output_vec = _vector3f_t()
    
    _call_c_function('coordinate_transform_omni',
                    ctypes.byref(c_output_vec),
                    to_frame.encode('utf-8'),
                    ctypes.byref(c_input_vec),
                    from_frame.encode('utf-8'))
    
    return _vector3f_to_numpy(c_output_vec)

def ned_to_enu(ned_vector: np.ndarray) -> np.ndarray:
    """Convert NED coordinates to ENU coordinates"""
    c_input = _numpy_to_vector3f(ned_vector)
    c_output = _vector3f_t()
    
    _call_c_function('ned_to_enu', ctypes.byref(c_output), ctypes.byref(c_input))
    return _vector3f_to_numpy(c_output)

def enu_to_ned(enu_vector: np.ndarray) -> np.ndarray:
    """Convert ENU coordinates to NED coordinates"""
    c_input = _numpy_to_vector3f(enu_vector)
    c_output = _vector3f_t()
    
    _call_c_function('enu_to_ned', ctypes.byref(c_output), ctypes.byref(c_input))
    return _vector3f_to_numpy(c_output)

# =============================================================================
# ROTATION CONVERSIONS
# =============================================================================

def rotation_matrix_to_quaternion(rotation_matrix: RotationMatrix) -> Quaternion:
    """Convert rotation matrix to quaternion"""
    c_matrix = rotation_matrix._to_c_struct()
    c_quat = _quaternion4f_t()
    
    _call_c_function('rotation_matrix_to_quaternion', ctypes.byref(c_quat), ctypes.byref(c_matrix))
    return Quaternion._from_c_struct(c_quat)

def quaternion_to_rotation_matrix(quaternion: Quaternion) -> RotationMatrix:
    """Convert quaternion to rotation matrix"""
    c_quat = quaternion._to_c_struct()
    c_matrix = _matrix3f_t()
    
    _call_c_function('quaternion_to_rotation_matrix', ctypes.byref(c_matrix), ctypes.byref(c_quat))
    return RotationMatrix._from_c_struct(c_matrix)

def euler_to_rotation_matrix(euler_angles: np.ndarray) -> RotationMatrix:
    """Convert Euler angles (roll, pitch, yaw) to rotation matrix"""
    c_euler = _numpy_to_vector3f(euler_angles)
    c_matrix = _matrix3f_t()
    
    _call_c_function('euler_angles_to_rotation_matrix', ctypes.byref(c_matrix), ctypes.byref(c_euler))
    return RotationMatrix._from_c_struct(c_matrix)

def rotation_matrix_to_euler(rotation_matrix: RotationMatrix) -> np.ndarray:
    """Convert rotation matrix to Euler angles (roll, pitch, yaw)"""
    c_matrix = rotation_matrix._to_c_struct()
    c_euler = _vector3f_t()
    
    _call_c_function('rotation_matrix_to_euler_angles', ctypes.byref(c_euler), ctypes.byref(c_matrix))
    return _vector3f_to_numpy(c_euler)

# =============================================================================
# EXPORTS
# =============================================================================

__all__ = [
    # Version
    'get_version',
    
    # Special classes
    'Quaternion', 'RotationMatrix', 'QuadrotorState', 'QuadrotorSetpoint',
    
    # Core controllers
    'geometric_control',
    
    # Robotics utilities
    'transform_world_to_body', 'transform_body_to_world', 'coordinate_transform',
    'ned_to_enu', 'enu_to_ned',
    
    # Rotation conversions
    'rotation_matrix_to_quaternion', 'quaternion_to_rotation_matrix',
    'euler_to_rotation_matrix', 'rotation_matrix_to_euler',
]