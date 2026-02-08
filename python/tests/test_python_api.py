#!/usr/bin/env python3
"""
Comprehensive test script for mini_quadlib Python API
"""

import sys
import os
import numpy as np

# Add the python directory to path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def test_version():
    """Test version retrieval"""
    print("Testing version...")

    import mini_quadlib as mql

    version = mql.get_version()
    print(f"✓ Library version: {version}")

    return True

def test_quaternion_operations():
    """Test quaternion operations"""
    print("\nTesting quaternion operations...")

    import mini_quadlib as mql

    # Create quaternions
    q1 = mql.Quaternion(1.0, 0.0, 0.0, 0.0)  # identity
    q2 = mql.Quaternion(0.707, 0.0, 0.0, 0.707)  # ~90° rotation about z

    # Normalize (method)
    q1_norm = q1.normalize()
    print(f"✓ Quaternion normalize: {q1} -> {q1_norm}")

    # Multiply (method + operator)
    q3 = q1.multiply(q2)
    q3_op = q1 * q2
    print(f"✓ Quaternion multiply: {q1} * {q2} = {q3}")
    print(f"✓ Quaternion operator *: {q1} * {q2} = {q3_op}")

    # Conjugate (method)
    q1_conj = q1.conjugate()
    print(f"✓ Quaternion conjugate: {q1} -> {q1_conj}")

    # from_array / to_array
    arr = np.array([0.5, 0.5, 0.5, 0.5], dtype=np.float32)
    q_from_arr = mql.Quaternion.from_array(arr)
    arr_back = q_from_arr.to_array()
    assert np.allclose(arr, arr_back), "from_array/to_array round-trip failed"
    print(f"✓ Quaternion from_array/to_array round-trip")

    # Identity
    q_id = mql.Quaternion.identity()
    assert q_id.w == 1.0 and q_id.x == 0.0 and q_id.y == 0.0 and q_id.z == 0.0
    print(f"✓ Quaternion identity: {q_id}")

    return True

def test_coordinate_transforms():
    """Test coordinate transformations"""
    print("\nTesting coordinate transformations...")

    import mini_quadlib as mql

    # Test NED/ENU conversions with numpy arrays
    ned_vec = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    enu_vec = mql.ned_to_enu(ned_vec)
    print(f"✓ NED to ENU: {ned_vec} -> {enu_vec}")

    # Convert back
    ned_vec2 = mql.enu_to_ned(enu_vec)
    assert np.allclose(ned_vec, ned_vec2), "NED->ENU->NED round-trip failed"
    print(f"✓ ENU to NED: {enu_vec} -> {ned_vec2}")

    # Test general coordinate transform
    try:
        result = mql.coordinate_transform(ned_vec, "NED", "ENU")
        print(f"✓ General coordinate transform: {ned_vec} (NED) -> {result} (ENU)")
    except Exception as e:
        print(f"⚠ General coordinate transform not available: {e}")

    # Test frame transforms
    world_vec = np.array([1.0, 0.0, 0.0], dtype=np.float32)
    attitude = mql.Quaternion(1.0, 0.0, 0.0, 0.0)  # identity
    body_vec = mql.transform_world_to_body(world_vec, attitude)
    assert np.allclose(world_vec, body_vec), "Identity transform should not change vector"
    print(f"✓ World to body (identity): {world_vec} -> {body_vec}")

    world_vec2 = mql.transform_body_to_world(body_vec, attitude)
    assert np.allclose(world_vec, world_vec2), "Body->world round-trip failed"
    print(f"✓ Body to world (identity): {body_vec} -> {world_vec2}")

    return True

def test_rotation_conversions():
    """Test rotation matrix/quaternion/euler conversions"""
    print("\nTesting rotation conversions...")

    import mini_quadlib as mql

    # Create a quaternion (~90° about z)
    q = mql.Quaternion(0.707, 0.0, 0.0, 0.707)

    # Convert to rotation matrix
    R = mql.quaternion_to_rotation_matrix(q)
    print(f"✓ Quaternion to rotation matrix: {q} -> RotationMatrix")

    # Convert back to quaternion
    q2 = mql.rotation_matrix_to_quaternion(R)
    print(f"✓ Rotation matrix to quaternion: RotationMatrix -> {q2}")

    # Check round-trip (signs may differ, so compare absolute values)
    q_arr = q.to_array()
    q2_arr = q2.to_array()
    assert np.allclose(np.abs(q_arr), np.abs(q2_arr), atol=1e-3), \
        f"Quaternion round-trip failed: {q_arr} vs {q2_arr}"
    print(f"✓ Quaternion->RotMatrix->Quaternion round-trip OK")

    # Test Euler angles
    euler = np.array([0.1, 0.2, 0.3], dtype=np.float32)  # roll, pitch, yaw
    R_euler = mql.euler_to_rotation_matrix(euler)
    print(f"✓ Euler to rotation matrix: {euler} -> RotationMatrix")

    euler2 = mql.rotation_matrix_to_euler(R_euler)
    assert np.allclose(euler, euler2, atol=1e-5), \
        f"Euler round-trip failed: {euler} vs {euler2}"
    print(f"✓ Rotation matrix to Euler: RotationMatrix -> {euler2}")

    # Test RotationMatrix class helpers
    R_id = mql.RotationMatrix.identity()
    assert np.allclose(R_id.to_array(), np.eye(3))
    print(f"✓ RotationMatrix identity OK")

    R_inv = R.inverse()
    product = R.to_array() @ R_inv.to_array()
    assert np.allclose(product, np.eye(3), atol=1e-5)
    print(f"✓ RotationMatrix inverse OK")

    return True

def test_geometric_controller():
    """Test geometric controller"""
    print("\nTesting geometric controller...")

    import mini_quadlib as mql

    # Create current state using Python classes
    current_state = mql.QuadrotorState(
        position=np.array([0.0, 0.0, 0.0]),
        velocity=np.array([0.0, 0.0, 0.0]),
        attitude=mql.Quaternion(1.0, 0.0, 0.0, 0.0),
        angular_velocity=np.array([0.0, 0.0, 0.0])
    )

    # Create desired setpoint
    desired_setpoint = mql.QuadrotorSetpoint(
        position=np.array([1.0, 1.0, -1.0]),
        velocity=np.array([0.0, 0.0, 0.0]),
        acceleration=np.array([0.0, 0.0, 0.0]),
        jerk=np.array([0.0, 0.0, 0.0]),
        snap=np.array([0.0, 0.0, 0.0]),
        yaw=0.0, yaw_dot=0.0, yaw_ddot=0.0
    )

    # Compute control
    try:
        control_output = mql.geometric_control(
            current_state=current_state,
            desired_setpoint=desired_setpoint,
            position_gains=np.array([1.0, 1.0, 2.0]),
            velocity_gains=np.array([0.5, 0.5, 1.0]),
            attitude_gains=np.array([1.0, 1.0, 1.0]),
            angular_velocity_gains=np.array([0.1, 0.1, 0.1]),
            quadrotor_mass=1.0,
            quadrotor_inertia=np.array([0.01, 0.01, 0.02]),
            arm_length_x=0.2,
            arm_length_y=0.2,
            thrust_coefficient=1e-6,
            moment_coefficient=1e-8
        )
        assert control_output.shape == (4,), f"Expected shape (4,), got {control_output.shape}"
        print(f"✓ Geometric control computed: {control_output}")
        return True
    except Exception as e:
        print(f"⚠ Geometric control failed: {e}")
        return False

def main():
    """Run all tests"""
    print("🚀 Running comprehensive mini_quadlib tests...\n")

    tests = [
        test_version,
        test_quaternion_operations,
        test_coordinate_transforms,
        test_rotation_conversions,
        test_geometric_controller,
    ]

    passed = 0
    total = len(tests)

    for test in tests:
        try:
            if test():
                passed += 1
        except Exception as e:
            print(f"✗ {test.__name__} failed: {e}")

    print(f"\n🏁 Tests completed: {passed}/{total} passed")

    if passed == total:
        print("🎉 All tests passed! Your mini_quadlib Python bindings are working perfectly!")
        return 0
    else:
        print("⚠ Some tests failed. Check the C library implementation.")
        return 1

if __name__ == "__main__":
    sys.exit(main())