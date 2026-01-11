/**
 * @file test.c
 * @brief Simple test suite for mini_quadlib
 */

#include <stdio.h>
#include <math.h>
#include "mini_quadlib.h"

#define TEST_EPSILON 1e-6
#define TEST_PASS "\033[32m✓\033[0m"
#define TEST_FAIL "\033[31m✗\033[0m"

int tests_passed = 0;
int tests_failed = 0;

void assert_near(const char* name, double actual, double expected, double epsilon) {
    if (fabs(actual - expected) < epsilon) {
        printf("%s %s: %.6f ≈ %.6f\n", TEST_PASS, name, actual, expected);
        tests_passed++;
    } else {
        printf("%s %s: %.6f ≠ %.6f (diff: %.6e)\n", TEST_FAIL, name, actual, expected, fabs(actual - expected));
        tests_failed++;
    }
}

void assert_vector_near(const char* name, Vector3 actual, Vector3 expected, double epsilon) {
    int pass = (fabs(actual.x - expected.x) < epsilon) &&
               (fabs(actual.y - expected.y) < epsilon) &&
               (fabs(actual.z - expected.z) < epsilon);
    
    if (pass) {
        printf("%s %s: [%.6f, %.6f, %.6f]\n", TEST_PASS, name, actual.x, actual.y, actual.z);
        tests_passed++;
    } else {
        printf("%s %s: [%.6f, %.6f, %.6f] ≠ [%.6f, %.6f, %.6f]\n", 
               TEST_FAIL, name, actual.x, actual.y, actual.z, expected.x, expected.y, expected.z);
        tests_failed++;
    }
}

void test_quaternion_operations(void) {
    printf("\n=== Testing Quaternion Operations ===\n");
    
    /* Test quaternion normalization */
    Quaternion q = quaternion_create(1.0, 1.0, 1.0, 1.0);
    Quaternion q_norm = quaternion_normalize(q);
    double norm = sqrt(q_norm.w * q_norm.w + q_norm.x * q_norm.x + 
                       q_norm.y * q_norm.y + q_norm.z * q_norm.z);
    assert_near("Quaternion normalize", norm, 1.0, TEST_EPSILON);
    
    /* Test identity quaternion rotation */
    Quaternion q_id = quaternion_create(1.0, 0.0, 0.0, 0.0);
    Vector3 v = vector3_create(1.0, 2.0, 3.0);
    Vector3 v_rot = quaternion_rotate_vector(q_id, v);
    assert_vector_near("Identity rotation", v_rot, v, TEST_EPSILON);
    
    /* Test quaternion conjugate */
    Quaternion q_conj = quaternion_conjugate(q_id);
    assert_near("Conjugate w", q_conj.w, 1.0, TEST_EPSILON);
    assert_near("Conjugate x", q_conj.x, 0.0, TEST_EPSILON);
}

void test_euler_conversions(void) {
    printf("\n=== Testing Euler Angle Conversions ===\n");
    
    /* Test identity */
    EulerAngles e_zero = euler_create(0.0, 0.0, 0.0);
    Quaternion q = euler_to_quaternion(e_zero);
    EulerAngles e_recovered = quaternion_to_euler(q);
    
    assert_near("Zero roll", e_recovered.roll, 0.0, TEST_EPSILON);
    assert_near("Zero pitch", e_recovered.pitch, 0.0, TEST_EPSILON);
    assert_near("Zero yaw", e_recovered.yaw, 0.0, TEST_EPSILON);
    
    /* Test round-trip conversion */
    EulerAngles e_orig = euler_create(0.5, 0.3, 0.7);
    q = euler_to_quaternion(e_orig);
    e_recovered = quaternion_to_euler(q);
    
    assert_near("Round-trip roll", e_recovered.roll, e_orig.roll, TEST_EPSILON);
    assert_near("Round-trip pitch", e_recovered.pitch, e_orig.pitch, TEST_EPSILON);
    assert_near("Round-trip yaw", e_recovered.yaw, e_orig.yaw, TEST_EPSILON);
}

void test_rotation_matrix(void) {
    printf("\n=== Testing Rotation Matrix Operations ===\n");
    
    /* Test identity matrix */
    RotationMatrix R_id = rotation_matrix_identity();
    Vector3 v = vector3_create(1.0, 2.0, 3.0);
    Vector3 v_rot = rotation_matrix_rotate_vector(R_id, v);
    assert_vector_near("Identity matrix rotation", v_rot, v, TEST_EPSILON);
    
    /* Test matrix transpose */
    RotationMatrix R = rotation_matrix_identity();
    R.m[0][1] = 0.5;
    RotationMatrix R_T = rotation_matrix_transpose(R);
    assert_near("Transpose [0][1]", R_T.m[1][0], 0.5, TEST_EPSILON);
}

void test_vector_operations(void) {
    printf("\n=== Testing Vector Operations ===\n");
    
    Vector3 v1 = vector3_create(1.0, 0.0, 0.0);
    Vector3 v2 = vector3_create(0.0, 1.0, 0.0);
    
    /* Test cross product */
    Vector3 cross = vector3_cross(v1, v2);
    Vector3 expected = vector3_create(0.0, 0.0, 1.0);
    assert_vector_near("Cross product", cross, expected, TEST_EPSILON);
    
    /* Test dot product */
    double dot = vector3_dot(v1, v2);
    assert_near("Dot product", dot, 0.0, TEST_EPSILON);
    
    /* Test norm */
    Vector3 v = vector3_create(3.0, 4.0, 0.0);
    double norm = vector3_norm(v);
    assert_near("Vector norm", norm, 5.0, TEST_EPSILON);
    
    /* Test normalize */
    Vector3 v_norm = vector3_normalize(v);
    assert_near("Normalized x", v_norm.x, 0.6, TEST_EPSILON);
    assert_near("Normalized y", v_norm.y, 0.8, TEST_EPSILON);
}

void test_frame_conversions(void) {
    printf("\n=== Testing Frame Conversions ===\n");
    
    /* Test ENU to NED and back */
    Vector3 v_enu = vector3_create(1.0, 2.0, 3.0);
    Vector3 v_ned = convert_frame(v_enu, FRAME_ENU, FRAME_NED);
    Vector3 v_enu_back = convert_frame(v_ned, FRAME_NED, FRAME_ENU);
    assert_vector_near("ENU->NED->ENU", v_enu_back, v_enu, TEST_EPSILON);
    
    /* Test ENU to NWU and back */
    Vector3 v_nwu = convert_frame(v_enu, FRAME_ENU, FRAME_NWU);
    v_enu_back = convert_frame(v_nwu, FRAME_NWU, FRAME_ENU);
    assert_vector_near("ENU->NWU->ENU", v_enu_back, v_enu, TEST_EPSILON);
    
    /* Test same frame conversion */
    Vector3 v_same = convert_frame(v_enu, FRAME_ENU, FRAME_ENU);
    assert_vector_near("Same frame", v_same, v_enu, TEST_EPSILON);
}

void test_geometric_controller(void) {
    printf("\n=== Testing Geometric Controller ===\n");
    
    /* Test hover condition (at desired position with zero velocity) */
    QuadrotorState state;
    state.position = vector3_create(0.0, 0.0, 1.0);
    state.velocity = vector3_create(0.0, 0.0, 0.0);
    state.attitude = quaternion_create(1.0, 0.0, 0.0, 0.0);
    state.angular_vel = vector3_create(0.0, 0.0, 0.0);
    
    Vector3 desired_pos = vector3_create(0.0, 0.0, 1.0);  /* Same as current */
    Vector3 desired_vel = vector3_create(0.0, 0.0, 0.0);
    Vector3 desired_acc = vector3_create(0.0, 0.0, 0.0);
    double desired_yaw = 0.0;
    
    GeometricControllerGains gains = geometric_controller_default_gains();
    ControlCommand cmd = geometric_controller_compute(
        state, desired_pos, desired_vel, desired_acc, desired_yaw, gains
    );
    
    /* In hover, thrust should equal weight (mass * gravity) */
    double expected_hover_thrust = gains.mass * gains.gravity;
    assert_near("Hover thrust", cmd.thrust, expected_hover_thrust, 0.1);
    
    /* Test upward motion command */
    desired_pos = vector3_create(0.0, 0.0, 2.0);  /* 1m above current */
    cmd = geometric_controller_compute(
        state, desired_pos, desired_vel, desired_acc, desired_yaw, gains
    );
    
    /* Thrust should be greater than hover thrust to go up */
    if (cmd.thrust > expected_hover_thrust) {
        printf("%s Upward thrust: %.2f N > %.2f N (hover)\n", TEST_PASS, cmd.thrust, expected_hover_thrust);
        tests_passed++;
    } else {
        printf("%s Upward thrust should be > hover thrust\n", TEST_FAIL);
        tests_failed++;
    }
}

int main(void) {
    printf("Mini Quadlib Test Suite\n");
    printf("=======================\n");
    
    test_quaternion_operations();
    test_euler_conversions();
    test_rotation_matrix();
    test_vector_operations();
    test_frame_conversions();
    test_geometric_controller();
    
    printf("\n=======================\n");
    printf("Tests passed: %d\n", tests_passed);
    printf("Tests failed: %d\n", tests_failed);
    printf("=======================\n");
    
    return tests_failed > 0 ? 1 : 0;
}
