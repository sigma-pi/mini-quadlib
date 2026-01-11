/**
 * @file example.c
 * @brief Example usage of mini_quadlib
 */

#include <stdio.h>
#include "mini_quadlib.h"

void print_vector3(const char* name, Vector3 v) {
    printf("%s: [%.4f, %.4f, %.4f]\n", name, v.x, v.y, v.z);
}

void print_quaternion(const char* name, Quaternion q) {
    printf("%s: w=%.4f, x=%.4f, y=%.4f, z=%.4f\n", name, q.w, q.x, q.y, q.z);
}

void print_euler(const char* name, EulerAngles e) {
    printf("%s: roll=%.4f°, pitch=%.4f°, yaw=%.4f°\n", 
           name, e.roll * 180.0 / M_PI, e.pitch * 180.0 / M_PI, e.yaw * 180.0 / M_PI);
}

void example_coordinate_conversions(void) {
    printf("\n=== Coordinate Conversions Example ===\n");
    
    /* Create Euler angles: 30° roll, 20° pitch, 45° yaw */
    EulerAngles euler = euler_create(30.0 * M_PI / 180.0, 
                                      20.0 * M_PI / 180.0, 
                                      45.0 * M_PI / 180.0);
    print_euler("Original Euler angles", euler);
    
    /* Convert to quaternion */
    Quaternion quat = euler_to_quaternion(euler);
    print_quaternion("Quaternion", quat);
    
    /* Convert quaternion back to Euler */
    EulerAngles euler_recovered = quaternion_to_euler(quat);
    print_euler("Recovered Euler angles", euler_recovered);
    
    /* Convert to rotation matrix */
    RotationMatrix R = quaternion_to_rotation_matrix(quat);
    printf("\nRotation Matrix:\n");
    for (int i = 0; i < 3; i++) {
        printf("  [%.4f, %.4f, %.4f]\n", R.m[i][0], R.m[i][1], R.m[i][2]);
    }
    
    /* Rotate a vector */
    Vector3 v = vector3_create(1.0, 0.0, 0.0);
    Vector3 v_rotated = quaternion_rotate_vector(quat, v);
    print_vector3("\nOriginal vector", v);
    print_vector3("Rotated vector", v_rotated);
}

void example_frame_conversions(void) {
    printf("\n=== Frame Conversions Example ===\n");
    
    /* Vector in ENU frame */
    Vector3 v_enu = vector3_create(1.0, 2.0, 3.0);
    print_vector3("Vector in ENU", v_enu);
    
    /* Convert to NED */
    Vector3 v_ned = convert_frame(v_enu, FRAME_ENU, FRAME_NED);
    print_vector3("Vector in NED", v_ned);
    
    /* Convert to NWU */
    Vector3 v_nwu = convert_frame(v_enu, FRAME_ENU, FRAME_NWU);
    print_vector3("Vector in NWU", v_nwu);
    
    /* Convert to FRD */
    Vector3 v_frd = convert_frame(v_enu, FRAME_ENU, FRAME_FRD);
    print_vector3("Vector in FRD", v_frd);
    
    /* Convert NED back to ENU */
    Vector3 v_enu_recovered = convert_frame(v_ned, FRAME_NED, FRAME_ENU);
    print_vector3("Vector back to ENU", v_enu_recovered);
}

void example_geometric_controller(void) {
    printf("\n=== Geometric Controller Example ===\n");
    
    /* Initialize quadrotor state */
    QuadrotorState state;
    state.position = vector3_create(0.0, 0.0, 1.0);  /* hovering at 1m height */
    state.velocity = vector3_create(0.0, 0.0, 0.0);
    state.attitude = quaternion_create(1.0, 0.0, 0.0, 0.0);  /* identity (level) */
    state.angular_vel = vector3_create(0.0, 0.0, 0.0);
    
    /* Desired state: move to position (1, 1, 2) */
    Vector3 desired_pos = vector3_create(1.0, 1.0, 2.0);
    Vector3 desired_vel = vector3_create(0.0, 0.0, 0.0);
    Vector3 desired_acc = vector3_create(0.0, 0.0, 0.0);
    double desired_yaw = 0.0;
    
    /* Get default controller gains */
    GeometricControllerGains gains = geometric_controller_default_gains();
    
    /* Compute control command */
    ControlCommand cmd = geometric_controller_compute(
        state, desired_pos, desired_vel, desired_acc, desired_yaw, gains
    );
    
    printf("Current position: [%.2f, %.2f, %.2f]\n", 
           state.position.x, state.position.y, state.position.z);
    printf("Desired position: [%.2f, %.2f, %.2f]\n", 
           desired_pos.x, desired_pos.y, desired_pos.z);
    printf("\nControl Command:\n");
    printf("  Thrust: %.4f N\n", cmd.thrust);
    print_vector3("  Moment", cmd.moment);
}

void example_vector_operations(void) {
    printf("\n=== Vector Operations Example ===\n");
    
    Vector3 v1 = vector3_create(1.0, 2.0, 3.0);
    Vector3 v2 = vector3_create(4.0, 5.0, 6.0);
    
    print_vector3("v1", v1);
    print_vector3("v2", v2);
    
    Vector3 v_sum = vector3_add(v1, v2);
    print_vector3("v1 + v2", v_sum);
    
    Vector3 v_diff = vector3_subtract(v1, v2);
    print_vector3("v1 - v2", v_diff);
    
    double dot_product = vector3_dot(v1, v2);
    printf("v1 · v2 = %.4f\n", dot_product);
    
    Vector3 cross_product = vector3_cross(v1, v2);
    print_vector3("v1 × v2", cross_product);
    
    double norm = vector3_norm(v1);
    printf("|v1| = %.4f\n", norm);
    
    Vector3 v_normalized = vector3_normalize(v1);
    print_vector3("normalized v1", v_normalized);
}

int main(void) {
    printf("Mini Quadlib Examples\n");
    printf("=====================\n");
    
    example_coordinate_conversions();
    example_frame_conversions();
    example_vector_operations();
    example_geometric_controller();
    
    printf("\n=== All examples completed successfully! ===\n");
    
    return 0;
}
