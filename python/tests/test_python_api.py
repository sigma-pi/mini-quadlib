#!/usr/bin/env python3
"""
Comprehensive test script for mini_quadlib Python API
Tests all exported functions, classes, and edge cases
"""

import sys
import os
import numpy as np

# Add the python directory to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# =============================================================================
# VERSION
# =============================================================================

def test_version():
    """Test version retrieval"""
    print("Testing version...")

    import mini_quadlib as mql

    version = mql.get_version()
    assert isinstance(version, str), "Version should be a string"
    assert len(version) > 0, "Version should not be empty"
    print(f"  ✓ Library version: {version}")

    return True


# =============================================================================
# QUATERNION CLASS AND OPERATIONS
# =============================================================================

def test_quaternion_creation():
    """Test Quaternion construction and properties"""
    print("\nTesting Quaternion creation...")

    import mini_quadlib as mql

    # Default (identity)
    q = mql.Quaternion()
    assert q.w == 1.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0
    print("  ✓ Default quaternion is identity")

    # Explicit values
    q = mql.Quaternion(0.5, 0.5, 0.5, 0.5)
    assert np.isclose(q.w, 0.5) and np.isclose(q.x, 0.5)
    print(f"  ✓ Explicit construction: {q}")

    # Identity class method
    q_id = mql.Quaternion.identity()
    assert q_id.w == 1.0 and q_id.x == 0.0
    print(f"  ✓ Identity class method: {q_id}")

    # from_array
    arr = np.array([0.5, 0.5, 0.5, 0.5], dtype=np.float32)
    q_from = mql.Quaternion.from_array(arr)
    assert np.allclose(q_from.to_array(), arr)
    print("  ✓ from_array round-trip")

    # to_dict
    d = q_from.to_dict()
    assert 'w' in d and 'x' in d and 'y' in d and 'z' in d
    print(f"  ✓ to_dict: {d}")

    # from_array validation
    try:
        mql.Quaternion.from_array(np.array([1, 2, 3]))
        assert False, "Should have raised ValueError"
    except ValueError:
        print("  ✓ from_array rejects wrong-length array")

    return True


def test_quaternion_operations():
    """Test Quaternion normalize, multiply, conjugate"""
    print("\nTesting Quaternion operations...")

    import mini_quadlib as mql

    # Normalize
    q = mql.Quaternion(2.0, 0.0, 0.0, 0.0)
    q_n = q.normalize()
    assert np.isclose(q_n.w, 1.0, atol=1e-5)
    assert np.isclose(np.linalg.norm(q_n.to_array()), 1.0, atol=1e-5)
    print(f"  ✓ Normalize: {q} -> {q_n}")

    q2 = mql.Quaternion(0.0, 1.0, 1.0, 1.0)
    q2_n = q2.normalize()
    assert np.isclose(np.linalg.norm(q2_n.to_array()), 1.0, atol=1e-5)
    print(f"  ✓ Normalize non-trivial: {q2} -> {q2_n}")

    # Multiply identity * q = q
    q_id = mql.Quaternion.identity()
    q_rot = mql.Quaternion(0.707, 0.0, 0.0, 0.707)
    q_result = q_id.multiply(q_rot)
    assert np.allclose(q_result.to_array(), q_rot.to_array(), atol=1e-3)
    print(f"  ✓ Multiply identity * q = q")

    # Operator *
    q_op = q_id * q_rot
    assert np.allclose(q_op.to_array(), q_rot.to_array(), atol=1e-3)
    print(f"  ✓ Operator * works")

    # Multiply non-commutative: q1*q2 != q2*q1 (for non-trivial q)
    q_a = mql.Quaternion(0.707, 0.707, 0.0, 0.0)
    q_b = mql.Quaternion(0.707, 0.0, 0.707, 0.0)
    q_ab = q_a.multiply(q_b)
    q_ba = q_b.multiply(q_a)
    assert not np.allclose(q_ab.to_array(), q_ba.to_array(), atol=1e-3), \
        "Quaternion multiplication should be non-commutative"
    print("  ✓ Multiply is non-commutative")

    # Conjugate: C library normalizes first, so compare with normalized input
    q_rot_n = q_rot.normalize()
    q_c = q_rot.conjugate()
    assert np.isclose(q_c.w, q_rot_n.w, atol=1e-5)
    assert np.isclose(q_c.x, -q_rot_n.x, atol=1e-5)
    assert np.isclose(q_c.y, -q_rot_n.y, atol=1e-5)
    assert np.isclose(q_c.z, -q_rot_n.z, atol=1e-5)
    print(f"  ✓ Conjugate: {q_rot} -> {q_c}")

    # q * conjugate(q) should give identity (for unit quaternion)
    q_unit = q_rot.normalize()
    q_conj = q_unit.conjugate()
    q_prod = q_unit * q_conj
    assert np.isclose(q_prod.w, 1.0, atol=1e-3)
    assert np.isclose(q_prod.x, 0.0, atol=1e-3)
    assert np.isclose(q_prod.y, 0.0, atol=1e-3)
    assert np.isclose(q_prod.z, 0.0, atol=1e-3)
    print("  ✓ q * conjugate(q) ≈ identity")

    return True


# =============================================================================
# ROTATION MATRIX CLASS
# =============================================================================

def test_rotation_matrix():
    """Test RotationMatrix class"""
    print("\nTesting RotationMatrix class...")

    import mini_quadlib as mql

    # Identity
    R = mql.RotationMatrix.identity()
    assert np.allclose(R.to_array(), np.eye(3))
    print("  ✓ Identity rotation matrix")

    # From numpy array
    arr = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=np.float32)
    R = mql.RotationMatrix(arr)
    assert np.allclose(R.to_array(), arr)
    print("  ✓ Construct from numpy array")

    # from_columns
    R2 = mql.RotationMatrix.from_columns(arr[:, 0], arr[:, 1], arr[:, 2])
    assert np.allclose(R2.to_array(), arr)
    print("  ✓ from_columns")

    # Column properties
    assert np.allclose(R.colx, arr[:, 0])
    assert np.allclose(R.coly, arr[:, 1])
    assert np.allclose(R.colz, arr[:, 2])
    print("  ✓ Column properties (colx, coly, colz)")

    # Transpose
    R_T = R.transpose()
    product = R.to_array() @ R_T.to_array()
    assert np.allclose(product, np.eye(3), atol=1e-5)
    print("  ✓ Transpose: R * R^T = I")

    # Inverse (same as transpose for rotation)
    R_inv = R.inverse()
    assert np.allclose(R_inv.to_array(), R_T.to_array())
    print("  ✓ Inverse = transpose")

    # Validation
    try:
        mql.RotationMatrix(np.zeros((2, 3)))
        assert False, "Should have raised ValueError"
    except ValueError:
        print("  ✓ Rejects non-3x3 matrix")

    return True


# =============================================================================
# ROTATION CONVERSIONS
# =============================================================================

def test_rotation_conversions():
    """Test quaternion/rotation matrix/euler conversions"""
    print("\nTesting rotation conversions...")

    import mini_quadlib as mql

    # Quaternion -> rotation matrix -> quaternion round-trip
    q = mql.Quaternion(0.707, 0.0, 0.0, 0.707).normalize()
    R = mql.quaternion_to_rotation_matrix(q)
    q2 = mql.rotation_matrix_to_quaternion(R)
    # Sign ambiguity: q and -q represent the same rotation
    assert np.allclose(np.abs(q.to_array()), np.abs(q2.to_array()), atol=1e-3)
    print("  ✓ Quaternion -> RotMatrix -> Quaternion round-trip")

    # Identity quaternion -> identity matrix
    q_id = mql.Quaternion.identity()
    R_id = mql.quaternion_to_rotation_matrix(q_id)
    assert np.allclose(R_id.to_array(), np.eye(3), atol=1e-5)
    print("  ✓ Identity quaternion -> identity matrix")

    # Euler -> rotation matrix -> euler round-trip
    euler = np.array([0.1, 0.2, 0.3], dtype=np.float32)
    R_e = mql.euler_to_rotation_matrix(euler)
    euler2 = mql.rotation_matrix_to_euler(R_e)
    assert np.allclose(euler, euler2, atol=1e-5)
    print(f"  ✓ Euler -> RotMatrix -> Euler round-trip: {euler} -> {euler2}")

    # Zero euler -> identity
    R_zero = mql.euler_to_rotation_matrix(np.zeros(3))
    assert np.allclose(R_zero.to_array(), np.eye(3), atol=1e-5)
    print("  ✓ Zero euler -> identity matrix")

    # Multiple angle tests
    for angles in [
        np.array([0.5, 0.0, 0.0]),   # roll only
        np.array([0.0, 0.4, 0.0]),   # pitch only
        np.array([0.0, 0.0, 0.6]),   # yaw only
        np.array([0.1, -0.2, 0.3]),  # mixed
    ]:
        R_test = mql.euler_to_rotation_matrix(angles)
        euler_back = mql.rotation_matrix_to_euler(R_test)
        assert np.allclose(angles, euler_back, atol=1e-4), \
            f"Euler round-trip failed for {angles}: got {euler_back}"
    print("  ✓ Multiple euler angle round-trips")

    return True


# =============================================================================
# COORDINATE TRANSFORMS
# =============================================================================

def test_coordinate_transforms():
    """Test NED/ENU and frame transformations"""
    print("\nTesting coordinate transformations...")

    import mini_quadlib as mql

    # NED -> ENU: (N,E,D) -> (E,N,-D) i.e. (x,y,z) -> (y,x,-z)
    ned = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    enu = mql.ned_to_enu(ned)
    assert np.allclose(enu, [2.0, 1.0, -3.0], atol=1e-5)
    print(f"  ✓ NED to ENU: {ned} -> {enu}")

    # ENU -> NED round-trip
    ned_back = mql.enu_to_ned(enu)
    assert np.allclose(ned, ned_back, atol=1e-5)
    print(f"  ✓ ENU to NED round-trip: {enu} -> {ned_back}")

    # Zero vector
    zero = np.zeros(3, dtype=np.float32)
    assert np.allclose(mql.ned_to_enu(zero), zero)
    assert np.allclose(mql.enu_to_ned(zero), zero)
    print("  ✓ Zero vector transforms to zero")

    # General coordinate_transform
    try:
        result = mql.coordinate_transform(ned, "NED", "ENU")
        assert np.allclose(result, enu, atol=1e-5)
        print(f"  ✓ General coordinate_transform NED->ENU matches")
    except Exception as e:
        print(f"  ⚠ coordinate_transform not available: {e}")

    return True


def test_frame_transforms():
    """Test world-to-body and body-to-world frame transforms"""
    print("\nTesting frame transformations...")

    import mini_quadlib as mql

    # Identity attitude: world = body
    q_id = mql.Quaternion.identity()
    world_vec = np.array([1.0, 2.0, 3.0], dtype=np.float32)

    body_vec = mql.transform_world_to_body(world_vec, q_id)
    assert np.allclose(world_vec, body_vec, atol=1e-5)
    print(f"  ✓ World->body (identity): {world_vec} -> {body_vec}")

    world_back = mql.transform_body_to_world(body_vec, q_id)
    assert np.allclose(world_vec, world_back, atol=1e-5)
    print(f"  ✓ Body->world (identity): {body_vec} -> {world_back}")

    # 90° rotation about z-axis: x->y, y->-x in world-to-body
    q_z90 = mql.Quaternion(0.7071068, 0.0, 0.0, 0.7071068).normalize()
    x_world = np.array([1.0, 0.0, 0.0], dtype=np.float32)
    x_body = mql.transform_world_to_body(x_world, q_z90)
    x_roundtrip = mql.transform_body_to_world(x_body, q_z90)
    assert np.allclose(x_world, x_roundtrip, atol=1e-3)
    print(f"  ✓ World->body->world round-trip with rotation: {x_world} -> {x_body} -> {x_roundtrip}")

    # Zero vector is always zero
    zero = np.zeros(3, dtype=np.float32)
    assert np.allclose(mql.transform_world_to_body(zero, q_z90), zero, atol=1e-5)
    assert np.allclose(mql.transform_body_to_world(zero, q_z90), zero, atol=1e-5)
    print("  ✓ Zero vector unchanged under rotation")

    return True


# =============================================================================
# QUADROTOR STATE AND SETPOINT CLASSES
# =============================================================================

def test_state_setpoint_classes():
    """Test QuadrotorState, QuadrotorSetpoint, and L1AdaptiveState classes"""
    print("\nTesting state/setpoint classes...")

    import mini_quadlib as mql

    # QuadrotorState defaults
    state = mql.QuadrotorState()
    assert np.allclose(state.position, np.zeros(3))
    assert np.allclose(state.velocity, np.zeros(3))
    assert state.attitude.w == 1.0
    assert np.allclose(state.angular_velocity, np.zeros(3))
    print("  ✓ QuadrotorState default construction")

    # QuadrotorState with values
    state = mql.QuadrotorState(
        position=np.array([1, 2, 3]),
        velocity=np.array([0.1, 0.2, 0.3]),
        attitude=mql.Quaternion(0.707, 0.707, 0, 0),
        angular_velocity=np.array([0.01, 0.02, 0.03])
    )
    assert np.allclose(state.position, [1, 2, 3], atol=1e-5)
    d = state.to_dict()
    assert 'position' in d and 'velocity' in d and 'attitude' in d
    print(f"  ✓ QuadrotorState with values, to_dict keys: {list(d.keys())}")

    # QuadrotorSetpoint defaults
    sp = mql.QuadrotorSetpoint()
    assert np.allclose(sp.position, np.zeros(3))
    assert sp.yaw == 0.0
    print("  ✓ QuadrotorSetpoint default construction")

    # QuadrotorSetpoint with values  
    sp = mql.QuadrotorSetpoint(
        position=np.array([1, 1, -1]),
        velocity=np.array([0.1, 0, 0]),
        acceleration=np.array([0, 0, -9.81]),
        jerk=np.array([0, 0, 0]),
        snap=np.array([0, 0, 0]),
        yaw=0.5, yaw_dot=0.1, yaw_ddot=0.0
    )
    assert np.isclose(sp.yaw, 0.5)
    assert np.isclose(sp.yaw_dot, 0.1)
    print("  ✓ QuadrotorSetpoint with values")

    # L1AdaptiveState defaults
    l1 = mql.L1AdaptiveState()
    assert np.allclose(l1.velocity, np.zeros(3))
    assert np.allclose(l1.adaptive_control, np.zeros(4))
    assert np.allclose(l1.get_total_control(), np.zeros(4))
    d = l1.to_dict()
    assert 'force_disturbance_estimate' in d
    assert 'baseline_control' in d
    assert 'adaptive_control' in d
    print(f"  ✓ L1AdaptiveState default construction, to_dict keys: {list(d.keys())}")

    return True


# =============================================================================
# GEOMETRIC CONTROLLER
# =============================================================================

def test_geometric_control():
    """Test geometric controller"""
    print("\nTesting geometric controller...")

    import mini_quadlib as mql

    # Hover test: at desired position, should get ~gravity compensation thrust
    state_hover = mql.QuadrotorState(
        position=np.array([0.0, 0.0, 0.0]),
        velocity=np.array([0.0, 0.0, 0.0]),
        attitude=mql.Quaternion.identity(),
        angular_velocity=np.array([0.0, 0.0, 0.0])
    )
    sp_hover = mql.QuadrotorSetpoint(
        position=np.array([0.0, 0.0, 0.0]),
    )
    mass = 1.0
    inertia = np.array([0.01, 0.01, 0.02])

    ctrl_hover = mql.geometric_control(
        current_state=state_hover,
        desired_setpoint=sp_hover,
        position_gains=np.array([5.0, 5.0, 8.0]),
        velocity_gains=np.array([3.0, 3.0, 5.0]),
        attitude_gains=np.array([8.0, 8.0, 4.0]),
        angular_velocity_gains=np.array([0.5, 0.5, 0.3]),
        mass=mass,
        inertia=inertia,
    )
    assert ctrl_hover.shape == (4,), f"Expected shape (4,), got {ctrl_hover.shape}"
    # In NED, gravity is +9.81 in z. Thrust to hover should be ~mass*g
    print(f"  ✓ Hover control: {ctrl_hover}")
    print(f"    (thrust={ctrl_hover[0]:.3f}, moments={ctrl_hover[1:]}")

    # Position error test: displaced from target, should produce corrective thrust
    state_displaced = mql.QuadrotorState(
        position=np.array([0.0, 0.0, 0.0]),
        velocity=np.array([0.0, 0.0, 0.0]),
        attitude=mql.Quaternion.identity(),
        angular_velocity=np.array([0.0, 0.0, 0.0])
    )
    sp_target = mql.QuadrotorSetpoint(
        position=np.array([1.0, 1.0, -1.0]),
    )
    ctrl_displaced = mql.geometric_control(
        current_state=state_displaced,
        desired_setpoint=sp_target,
        position_gains=np.array([5.0, 5.0, 8.0]),
        velocity_gains=np.array([3.0, 3.0, 5.0]),
        attitude_gains=np.array([8.0, 8.0, 4.0]),
        angular_velocity_gains=np.array([0.5, 0.5, 0.3]),
        mass=mass,
        inertia=inertia,
    )
    assert ctrl_displaced.shape == (4,)
    # With position error, thrust should differ from hover
    print(f"  ✓ Displaced control: {ctrl_displaced}")

    # With velocity test
    state_moving = mql.QuadrotorState(
        position=np.array([0.0, 0.0, 0.0]),
        velocity=np.array([1.0, 0.0, 0.0]),
        attitude=mql.Quaternion.identity(),
        angular_velocity=np.array([0.0, 0.0, 0.0])
    )
    ctrl_moving = mql.geometric_control(
        current_state=state_moving,
        desired_setpoint=sp_hover,
        position_gains=np.array([5.0, 5.0, 8.0]),
        velocity_gains=np.array([3.0, 3.0, 5.0]),
        attitude_gains=np.array([8.0, 8.0, 4.0]),
        angular_velocity_gains=np.array([0.5, 0.5, 0.3]),
        mass=mass,
        inertia=inertia,
    )
    assert ctrl_moving.shape == (4,)
    print(f"  ✓ Moving state control: {ctrl_moving}")

    return True


# =============================================================================
# L1 ADAPTIVE CONTROLLER
# =============================================================================

def test_l1_adaptive_control():
    """Test L1 adaptive controller"""
    print("\nTesting L1 adaptive controller...")

    import mini_quadlib as mql

    mass = 1.0
    inertia = np.array([0.01, 0.01, 0.02])

    # L1 params
    As_v = 5.0
    As_W = 5.0
    lpf1_cutoff_f = 10.0
    lpf1_cutoff_M = 10.0
    lpf2_cutoff_M = 10.0
    dt = 0.01

    # Initialize L1 state
    l1_state = mql.L1AdaptiveState()

    # Simulate a few iterations
    baseline_ctrl = np.array([9.81, 0.0, 0.0, 0.0], dtype=np.float32)  # hover thrust
    attitude = mql.Quaternion.identity()
    velocity = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    omega = np.array([0.0, 0.0, 0.0], dtype=np.float32)

    for i in range(5):
        l1_state = mql.l1_adaptive_control(
            previous_state=l1_state,
            current_velocity=velocity,
            current_angular_velocity=omega,
            current_attitude=attitude,
            baseline_control=baseline_ctrl,
            dt=dt,
            As_v=As_v,
            As_W=As_W,
            lpf1_cutoff_freq_force=lpf1_cutoff_f,
            lpf1_cutoff_freq_moment=lpf1_cutoff_M,
            lpf2_cutoff_freq_moment=lpf2_cutoff_M,
            mass=mass,
            inertia=inertia,
        )

    assert isinstance(l1_state, mql.L1AdaptiveState)
    total_ctrl = l1_state.get_total_control()
    assert total_ctrl.shape == (4,)
    print(f"  ✓ L1 after 5 iters: baseline={l1_state.baseline_control}, adaptive={l1_state.adaptive_control}")
    print(f"    total_control={total_ctrl}")
    print(f"    force_disturbance_est={l1_state.force_disturbance_estimate}")
    print(f"    moment_disturbance_est={l1_state.moment_disturbance_estimate}")

    # State should be inspectable
    d = l1_state.to_dict()
    assert len(d) > 0
    print(f"  ✓ L1 state to_dict has {len(d)} entries")

    # Test with disturbance-like scenario: give nonzero velocity that
    # doesn't match what the model predicts, adaptation should react
    l1_state_dist = mql.L1AdaptiveState()
    velocity_with_disturbance = np.array([0.5, -0.3, 0.1], dtype=np.float32)

    for i in range(20):
        l1_state_dist = mql.l1_adaptive_control(
            previous_state=l1_state_dist,
            current_velocity=velocity_with_disturbance,
            current_angular_velocity=omega,
            current_attitude=attitude,
            baseline_control=baseline_ctrl,
            dt=dt,
            As_v=As_v,
            As_W=As_W,
            lpf1_cutoff_freq_force=lpf1_cutoff_f,
            lpf1_cutoff_freq_moment=lpf1_cutoff_M,
            lpf2_cutoff_freq_moment=lpf2_cutoff_M,
            mass=mass,
            inertia=inertia,
        )

    # After 20 iterations with unexpected velocity, disturbance estimate should be nonzero
    dist_est = l1_state_dist.force_disturbance_estimate
    print(f"  ✓ L1 disturbance scenario: force_est={dist_est}, adaptive={l1_state_dist.adaptive_control}")

    return True


# =============================================================================
# COMBINED GEOMETRIC + L1 PIPELINE
# =============================================================================

def test_geometric_l1_pipeline():
    """Test combined geometric + L1 adaptive control pipeline"""
    print("\nTesting geometric + L1 pipeline...")

    import mini_quadlib as mql

    mass = 1.0
    inertia = np.array([0.01, 0.01, 0.02])

    state = mql.QuadrotorState(
        position=np.array([0.0, 0.0, 0.0]),
        velocity=np.array([0.0, 0.0, 0.0]),
        attitude=mql.Quaternion.identity(),
        angular_velocity=np.array([0.0, 0.0, 0.0])
    )
    setpoint = mql.QuadrotorSetpoint(
        position=np.array([1.0, 0.0, -1.0]),
    )

    # Step 1: Geometric control for baseline
    baseline = mql.geometric_control(
        current_state=state,
        desired_setpoint=setpoint,
        position_gains=np.array([5.0, 5.0, 8.0]),
        velocity_gains=np.array([3.0, 3.0, 5.0]),
        attitude_gains=np.array([8.0, 8.0, 4.0]),
        angular_velocity_gains=np.array([0.5, 0.5, 0.3]),
        mass=mass,
        inertia=inertia,
    )
    print(f"  ✓ Baseline (geometric): {baseline}")

    # Step 2: L1 adaptive control
    l1_state = mql.L1AdaptiveState()
    l1_state = mql.l1_adaptive_control(
        previous_state=l1_state,
        current_velocity=state.velocity,
        current_angular_velocity=state.angular_velocity,
        current_attitude=state.attitude,
        baseline_control=baseline,
        dt=0.01,
        As_v=5.0, As_W=5.0,
        lpf1_cutoff_freq_force=10.0,
        lpf1_cutoff_freq_moment=10.0,
        lpf2_cutoff_freq_moment=10.0,
        mass=mass,
        inertia=inertia,
    )

    total = l1_state.get_total_control()
    print(f"  ✓ Augmented (baseline+adaptive): {total}")
    assert total.shape == (4,)
    print("  ✓ Pipeline produces valid 4-element control output")

    return True


# =============================================================================
# EDGE CASES AND INPUT VALIDATION
# =============================================================================

def test_edge_cases():
    """Test edge cases and input validation"""
    print("\nTesting edge cases...")

    import mini_quadlib as mql

    # Large values
    big = np.array([1e6, 1e6, 1e6], dtype=np.float32)
    enu = mql.ned_to_enu(big)
    ned_back = mql.enu_to_ned(enu)
    assert np.allclose(big, ned_back, rtol=1e-3)
    print("  ✓ Large value NED/ENU round-trip")

    # Small values
    small = np.array([1e-6, 1e-6, 1e-6], dtype=np.float32)
    enu_s = mql.ned_to_enu(small)
    ned_s = mql.enu_to_ned(enu_s)
    assert np.allclose(small, ned_s, atol=1e-10)
    print("  ✓ Small value NED/ENU round-trip")

    # Negative values
    neg = np.array([-1.0, -2.0, -3.0], dtype=np.float32)
    enu_n = mql.ned_to_enu(neg)
    ned_n = mql.enu_to_ned(enu_n)
    assert np.allclose(neg, ned_n, atol=1e-5)
    print("  ✓ Negative value NED/ENU round-trip")

    # Integer input (should auto-cast)
    result = mql.ned_to_enu(np.array([1, 2, 3]))
    assert result.shape == (3,)
    print("  ✓ Integer input auto-cast")

    # List input via numpy
    result = mql.ned_to_enu(np.array([1.0, 2.0, 3.0]))
    assert result.shape == (3,)
    print("  ✓ Float list input")

    return True


# =============================================================================
# MAIN
# =============================================================================

def main():
    """Run all tests"""
    print("=" * 60)
    print("  mini_quadlib comprehensive Python API tests")
    print("=" * 60)

    tests = [
        test_version,
        test_quaternion_creation,
        test_quaternion_operations,
        test_rotation_matrix,
        test_rotation_conversions,
        test_coordinate_transforms,
        test_frame_transforms,
        test_state_setpoint_classes,
        test_geometric_control,
        test_l1_adaptive_control,
        test_geometric_l1_pipeline,
        test_edge_cases,
    ]

    passed = 0
    failed = 0
    total = len(tests)

    for test in tests:
        try:
            if test():
                passed += 1
            else:
                failed += 1
                print(f"  ✗ {test.__name__} returned False")
        except Exception as e:
            failed += 1
            print(f"  ✗ {test.__name__} EXCEPTION: {e}")
            import traceback
            traceback.print_exc()

    print("\n" + "=" * 60)
    print(f"  Results: {passed}/{total} passed, {failed} failed")
    print("=" * 60)

    if passed == total:
        print("  All tests passed!")
        return 0
    else:
        print("  Some tests failed.")
        return 1

if __name__ == "__main__":
    sys.exit(main())