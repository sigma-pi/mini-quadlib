#pragma once

// include/mini-quadlib.h
#ifndef MINI_QUADLIB_H
#define MINI_QUADLIB_H

// uncomment to enable NDEBUG by default
// #if !defined(DEBUG) && !defined(NDEBUG)
//     #define NDEBUG
// #endif

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file mini_quadlib.h
 * @brief Mini-QuadLib - Simple quadrotor control library
 * @author Chengyu Yang
 * @version 0.1.1
 * @date 2026
 * 
 * This library provides the geometric controller for quadrotor control,
 * along with utility functions for coordinate transformations and safety checks.
 */

// =============================================================================
// VERSION INFO
// =============================================================================

#define QUADLIB_VERSION_STRING "0.1.1"

/*
Return the version string of the mini-quadlib library as char* ended with '\0'.
*/
#define QUADLIB_VERSION() QUADLIB_VERSION_STRING

/*
Return the version string of the mini-quadlib library as char* ended with '\0'.
*/
static inline const char* quadlib_version(void) {
    return QUADLIB_VERSION_STRING;
}

// =============================================================================
// TYPES AND STRUCTURES: SPECIAL TYPE FOR STATUS CODES
// =============================================================================

/* 
Return success/error codes (can be used as int):
{
Success: 0
Warning: -1
Invalid input: -2
Not initialized: -3
Safety violation: -4
}
*/
typedef enum {
    QUADLIB_SUCCESS = 0,
    QUADLIB_WARNING = -1,
    QUADLIB_ERROR_INVALID_INPUT_NUMBER = -2,
    QUADLIB_ERROR_INVALID_INPUT_STRING = -3,
    QUADLIB_ERROR_SAFETY_VIOLATION = -4
} quadlib_result_t;

// =============================================================================
// TYPES AND STRUCTURES: BASIC DATA STRUCTURES
// =============================================================================

/*
Basic vector (float): 
{
float x
float y
float z
}
Preffered to be column vector for matrix operations
*/ 
typedef struct {
    float x, y, z;
} vector3f_t;

/*
Basic 3x3 matrix (float): 
{
vector3f_t colx
vector3f_t coly
vector3f_t colz
}
Representation:
| colx.x coly.x colz.x |
| colx.y coly.y colz.y |
| colx.z coly.z colz.z |
*/
typedef struct {
    vector3f_t colx;
    vector3f_t coly;
    vector3f_t colz;
} matrix3f_t;

/* 
Quaternion (float): 
{
float w
float x
float y
float z
}
*/ 
typedef struct {
    float w, x, y, z;
} quaternion4f_t;

/*
Controller output in 4D (float):
{
float u1
float u2
float u3
float u4
}
Most common interpretations:
Thrust and moments: [f, Mx, My, Mz]
Thrust and angular rates: [f, p, q, r]
Individual thrusts: [f1, f2, f3, f4]
*/
typedef struct {
    float u1, u2, u3, u4;
} control_4f_t;

/*
State of the quadrotor:
{
vector3f_t pos
vector3f_t vel
quaternion4f_t quat
vector3f_t omega
}
*/
typedef struct {
    vector3f_t pos;
    vector3f_t vel;
    quaternion4f_t quat;
    vector3f_t omega;
} state_t;

/*
Desired setpoint from planner:
{
vector3f_t pos
vector3f_t vel
vector3f_t acc
vector3f_t jerk
vector3f_t snap
float yaw
float yaw_dot
float yaw_ddot
}
*/
typedef struct {
    vector3f_t pos;
    vector3f_t vel;
    vector3f_t acc;
    vector3f_t jerk;
    vector3f_t snap;
    float yaw;
    float yaw_dot;
    float yaw_ddot;
} setpoint_t;

/*
Controller parameters for geometric controller:
{
vector3f_t k_p
vector3f_t k_v
vector3f_t k_R
vector3f_t k_O
}
*/
typedef struct {
    vector3f_t k_p;             // Position gains, x y z
    vector3f_t k_v;             // Velocity gains, x y z
    vector3f_t k_R;             // Attitude gains, x y z
    vector3f_t k_O;             // Angular velocity gains, x y z
} geometric_params_t;

/*
Parameters for quadrotor in "X" configuration:
{
vector3f_t inertia
float mass
float xlen
float ylen
float k_thrust
float k_drag
}
NED frame: X points up, Y points right, Z points into the plane
       X
       ^
3(CW)    1(CCW)
    \    /
     \/\/
     |__|      >Y
    /    \
   /      \
2(CCW)   4(CW)
*/
typedef struct {
    vector3f_t inertia;   // inertia around body frame axes [kg*m^2]
    float mass;           // mass of the drone [kg]
    float xlen;           // distance between motors on x axis [m]
    float ylen;           // distance between motors on y axis [m]
    float k_thrust;       // thrust coefficient [N/(rad/s)^2]
    float k_drag;         // drag (torque) coefficient [Nm/(rad/s)^2]
} quadx_params_t;

// =============================================================================
// CORE CONTROLLER FUNCTIONS: GEOMETRIC CONTROLLER
// =============================================================================

/**
 * @brief Geometric controller for quadrotor (NED), repquiring full parameter list
 * 
 * @param output_control_TM Pointer to store the output control (thrust and moments)
 * @param current_state Pointer to the current state of the quadrotor
 * @param desired_state Pointer to the desired setpoint state
 * @param dt Time step since last control update [s]
 * @param ctrl_params Pointer to the geometric controller parameters 
 * @param quad_params Pointer to the quadrotor physical parameters
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 * 
 * @example
 * @code{.c}
 *     QUADLIB_CHECK(geometric_controller_TM_fullparam(&output_control_TM,
 *                                                     &current_state,
 *                                                     &desired_state,
 *                                                     dt,
 *                                                     &params));
 * @endcode
 */
quadlib_result_t geometric_controller_TM_fullparam(control_4f_t* output_control_TM,
                                                    const state_t* current_state,
                                                    const setpoint_t* desired_state,
                                                    const geometric_params_t* ctrl_params,
                                                    const quadx_params_t* quad_params);

// =============================================================================
// UTILITY FUNCTIONS: ROBOTICS TOOLS
// =============================================================================

/**
 * @brief Transform vector from world frame to body frame using current attitude
 * 
 * @param output_body_vector Pointer to store the output vector in body frame
 * @param world_vector Pointer to the input vector in world frame
 * @param attitude Pointer to the current attitude quaternion
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t frame_world_to_body(vector3f_t* output_body_vector,
                                     const vector3f_t* world_vector,
                                     const quaternion4f_t* attitude);

/**
 * @brief Transform vector from body frame to world frame using current attitude
 * 
 * @param output_world_vector Pointer to store the output vector in world frame
 * @param body_vector Pointer to the input vector in body frame
 * @param attitude Pointer to the current attitude quaternion
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t frame_body_to_world(vector3f_t* output_world_vector,
                                     const vector3f_t* body_vector,
                                     const quaternion4f_t* attitude);

/**
 * @brief Transform coordinate vector from one frame to another, e.g., NED to ENU
 * 
 * @param output_coordinate_vector Pointer to store the output coordinate vector
 * @param to_frame Target frame string (e.g., "ENU", "NED", "NWU", "SEU")
 * @param input_coordinate_vector Pointer to the input coordinate vector
 * @param from_frame Source frame string (e.g., "ENU", "NED", "NWU", "SEU")
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 *       Only checks the first three characters of the frame strings, case-insensitive.
 *       Valid characters are: E (East), N (North), W (West), S (South), U (Up), D (Down),
 *                             e (east), n (north), w (west), s (south), u (up), d (down)
 */
quadlib_result_t coordinate_transform_omni(vector3f_t* output_coordinate_vector,
                                           const char* to_frame,
                                           const vector3f_t* input_coordinate_vector,
                                           const char* from_frame);

/**
 * @brief Transform coordinate vector from ENU to NED (quick simple instance of coordinate_transform_omni)
 * 
 * @param output_coordinate_vector Pointer to store the output coordinate vector
 * @param input_coordinate_vector Pointer to the input coordinate vector
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t enu_to_ned(vector3f_t* output_coordinate_vector,
                            const vector3f_t* input_coordinate_vector);

/**
 * @brief Transform coordinate vector from NED to ENU (quick simple instance of coordinate_transform_omni)
 * 
 * @param output_coordinate_vector Pointer to store the output coordinate vector
 * @param input_coordinate_vector Pointer to the input coordinate vector
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t ned_to_enu(vector3f_t* output_coordinate_vector,
                            const vector3f_t* input_coordinate_vector);
                                
/**
 * @brief Normalize a quaternion to unit length
 * 
 * @param output_q Pointer to store the output normalized quaternion, (w, x, y, z)
 * @param q Pointer to the input quaternion, (w, x, y, z)
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t quaternion_normalize(quaternion4f_t* output_q, const quaternion4f_t* q);

/**
 * @brief Normalize a rotation matrix to be orthonormal
 * 
 * @param output_R Pointer to store the output normalized rotation matrix
 * @param R Pointer to the input rotation matrix
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t rotation_matrix_normalize(matrix3f_t* output_R, const matrix3f_t* R);

/**
 * @brief Normalize Euler angles to standard ranges (-pi, pi] radians
 * 
 * @param output_euler_angles Pointer to store the output normalized Euler angles, (roll, pitch, yaw)
 * @param euler_angles Pointer to the input Euler angles, (roll, pitch, yaw)
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t euler_angles_normalize(vector3f_t* output_euler_angles, const vector3f_t* euler_angles);

/**
 * @brief Hamilton product of two quaternions, NOT commutative! 
 *        Automatically normalizes the output quaternion
 * 
 * @param output_q Pointer to store the output quaternion result, (w, x, y, z)
 * @param q1 Pointer to the first quaternion (delta quaternion, following movement, (w, x, y, z))
 * @param q2 Pointer to the second quaternion (current attitude, (w, x, y, z))
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 *       'q followed by dq' is equivalent to 'dq * q', where '*' is quaternion multiplication
 */
quadlib_result_t quaternion_multiply(quaternion4f_t* output_q, const quaternion4f_t* q1, const quaternion4f_t* q2);

/**
 * @brief Compute the conjugate and normalize of a quaternion
 *        Automatically normalizes the output quaternion
 * 
 * @param output_q Pointer to store the output quaternion result, (w, x, y, z)
 * @param q Pointer to the input quaternion, (w, x, y, z)
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t quaternion_conjugate(quaternion4f_t* output_q, const quaternion4f_t* q);

/**
 * @brief Convert rotation matrix to quaternion
 * 
 * @param output_q Pointer to store the output quaternion, (w, x, y, z)
 * @param R Pointer to the input rotation matrix
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t rotation_matrix_to_quaternion(quaternion4f_t* output_q, const matrix3f_t* R);

/**
 * @brief Convert quaternion to rotation matrix
 * 
 * @param output_R Pointer to store the output rotation matrix
 * @param q Pointer to the input quaternion, (w, x, y, z)
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t quaternion_to_rotation_matrix(matrix3f_t* output_R, const quaternion4f_t* q);

/**
 * @brief Convert Euler angles (roll, pitch, yaw) radians to rotation matrix, R = Rz(yaw) * Ry(pitch) * Rx(roll)
 * 
 * @param output_R Pointer to store the output rotation matrix
 * @param euler_angles Pointer to the input Euler angles (roll, pitch, yaw)
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t euler_angles_to_rotation_matrix(matrix3f_t* output_R, const vector3f_t* euler_angles);

/**
 * @brief Convert rotation matrix to Euler angles (roll, pitch, yaw) radians, R = Rz(yaw) * Ry(pitch) * Rx(roll)
 * 
 * @param output_euler_angles Pointer to store the output Euler angles (roll, pitch, yaw)
 * @param R Pointer to the input rotation matrix
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t rotation_matrix_to_euler_angles(vector3f_t* output_euler_angles, const matrix3f_t* R);

/**
 * @brief Compute the transpose of a 3x3 rotation matrix.
 *        Automatically normalizes the output matrix, equivalent to computing the inverse of a rotation matrix.
 * 
 * @param output_RT Pointer to store the output transposed rotation matrix
 * @param R Pointer to the input rotation matrix
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t rotation_matrix_transpose(matrix3f_t* output_RT, const matrix3f_t* R);

// =============================================================================
// UTILITY FUNCTIONS: NORMAL MATHS
// =============================================================================

/**
 * @brief Determine if two 3D vectors are approximately equal within a small tolerance
 * 
 * @param is_equal Pointer to store the output boolean result (true if equal, false otherwise)
 * @param a Pointer to the first input vector
 * @param b Pointer to the second input vector
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t vector3_is_equalf(bool* is_equal, vector3f_t* a, const vector3f_t* b);

/**
 * @brief Subtract two 3D vectors: output_v = a - b
 * 
 * @param output_v Pointer to store the output vector
 * @param a Pointer to the first input vector
 * @param b Pointer to the second input vector
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t vector3_sub(vector3f_t* output_v, const vector3f_t* a, const vector3f_t* b);

/**
 * @brief Add two 3D vectors: output_v = a + b
 * 
 * @param output_v Pointer to store the output vector
 * @param a Pointer to the first input vector
 * @param b Pointer to the second input vector
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t vector3_add(vector3f_t* output_v, const vector3f_t* a, const vector3f_t* b);

/**
 * @brief Compute the norm (magnitude) of a 3D vector
 * 
 * @param output_norm Pointer to store the output norm
 * @param v Pointer to the input vector
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t vector3_get_norm(float* output_norm, const vector3f_t* v);

/**
 * @brief Normalize a 3D vector
 * 
 * @param output_v Pointer to store the output normalized vector
 * @param v Pointer to the input vector
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t vector3_normalize(vector3f_t* output_v, const vector3f_t* v);

/**
 * @brief Compute the dot product of two 3D vectors
 * 
 * @param output_dot Pointer to store the output dot product result
 * @param a Pointer to the first input vector (regarded as row vector)
 * @param b Pointer to the second input vector (regarded as column vector)
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t vector3_dot(float* output_dot, const vector3f_t* a, const vector3f_t* b);

/**
 * @brief Compute the dot product of two 3D vectors, output as a 3x3 matrix
 * 
 * @param output_dot Pointer to store the output dot product result as a 3x3 matrix
 * @param a Pointer to the first input vector (regarded as column vector)
 * @param b Pointer to the second input vector (regarded as row vector)
 * @return quadlib_result_t 
 */
quadlib_result_t vector3_dotcr(matrix3f_t* output_dot, const vector3f_t* a, const vector3f_t* b);

/**
 * @brief Compute the dot product of a 3D vector and a 3x3 matrix
 * 
 * @param output_dot Pointer to store the output dot product result
 * @param v Pointer to the input vector (regarded as row vector)
 * @param m Pointer to the input matrix
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t vector3_dotm(vector3f_t* output_dot, const vector3f_t* v, const matrix3f_t* m);

/**
* @brief Compute the cross product of two 3D vectors
* 
* @param output_cross Pointer to store the output cross product vector
* @param a Pointer to the first input vector
* @param b Pointer to the second input vector
* 
* @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
* 
* @note You can use macro QUADLIB_CHECK() to check the returned status.
*/
quadlib_result_t vector3_cross(vector3f_t* output_cross, const vector3f_t* a, const vector3f_t* b);

/**
 * @brief Compute the hat operator (skew-symmetric matrix) of a 3D vector
 * 
 * @param output_matrix Pointer to store the output skew-symmetric matrix
 * @param v Pointer to the input vector
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 *       Same as vector3_wedge(), but recommended to use vector3_hat().
 *       Definition:
 *       If v = [x, y, z]^T, then
 *       hat(v) = |  0  -z   y |
 *                |  z   0  -x |
 *                | -y   x   0 |
 */
quadlib_result_t vector3_hat(matrix3f_t* output_matrix, const vector3f_t* v);

/**
 * @brief Compute the hat operator (skew-symmetric matrix) of a 3D vector
 * 
 * @param output_matrix Pointer to store the output skew-symmetric matrix
 * @param v Pointer to the input vector
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 *       Same as vector3_hat() by simply calling it, recommended to use vector3_hat().
 *       Definition:
 *       If v = [x, y, z]^T, then
 *       hat(v) = |  0  -z   y |
 *                |  z   0  -x |
 *                | -y   x   0 |
 */
quadlib_result_t vector3_wedge(matrix3f_t* output_matrix, const vector3f_t* v);

/**
 * @brief Determine if two 3x3 matrices are approximately equal within a small tolerance
 * 
 * @param is_equal Pointer to store the output boolean result (true if equal, false otherwise)
 * @param a Pointer to the first input matrix
 * @param b Pointer to the second input matrix
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t matrix3_is_equalf(bool* is_equal, matrix3f_t* a, const matrix3f_t* b);

/**
 * @brief Compute the subtraction of two 3x3 matrices: output_dot = a - b
 * 
 * @param output_dot Pointer to store the output subtraction result as a 3x3 matrix
 * @param a Pointer to the first input matrix
 * @param b Pointer to the second input matrix
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t matrix3_sub(matrix3f_t* output_dot, const matrix3f_t* a, const matrix3f_t* b);

/**
 * @brief Compute the addition of two 3x3 matrices: output_dot = a + b
 * 
 * @param output_dot Pointer to store the output addition result as a 3x3 matrix
 * @param a Pointer to the first input matrix
 * @param b Pointer to the second input matrix
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t matrix3_add(matrix3f_t* output_dot, const matrix3f_t* a, const matrix3f_t* b);

/**
 * @brief Compute the dot product of two 3x3 matrices
 * 
 * @param output_dot Pointer to store the output dot product result as a 3x3 matrix
 * @param a Pointer to the first input matrix
 * @param b Pointer to the second input matrix
 * @return quadlib_result_t 
 */
quadlib_result_t matrix3_dot(matrix3f_t* output_dot, const matrix3f_t* a, const matrix3f_t* b);

/**
 * @brief Compute the dot product of a 3x3 matrix and a 3D vector
 * 
 * @param output_dot Pointer to store the output dot product result
 * @param m Pointer to the input matrix
 * @param v Pointer to the input vector (regarded as column vector)
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 */
quadlib_result_t matrix3_dotv(vector3f_t* output_dot, const matrix3f_t* m, const vector3f_t* v);

/**
 * @brief Compute the vee operator (inverse of hat) of a skew-symmetric matrix
 * 
 * @param output_v Pointer to store the output vector
 * @param M Pointer to the input skew-symmetric matrix
 * 
 * @return An enum type (quadlib_result_t) result of the controller computation, showing success or error code
 * 
 * @note You can use macro QUADLIB_CHECK() to check the returned status.
 *       Definition:
 *       If M = |  0  -z   y |
 *              |  z   0  -x |
 *              | -y   x   0 |,
 *       then vee(M) = [x, y, z]^T
 */
quadlib_result_t matrix3_vee(vector3f_t* output_v, const matrix3f_t* M);

// =============================================================================
// SAFETY CHECK FUNCTIONS
// =============================================================================

// bool safety_check_attitude(const quaternion4f_t* attitude, float max_tilt_rad);
// bool safety_check_rates(const vector3f_t* rates, const vector3f_t* max_rates);
// bool safety_check_position(const vector3f_t* position, const vector3f_t* min_pos, const vector3f_t* max_pos);
// control_4f_t safety_limit_control(const control_4f_t* control, const geometric_params_t* limits);

// =============================================================================
// SAFETY CHECK MACROS
// =============================================================================

/**
 * @brief Check if quadlib function call succeeded, return on error
 *        
 *        This macro executes the given function call and automatically handles errors.
 *        If the function fails, it prints an error message and returns the error code
 *        from the current function.
 * 
 * @param func_call The quadlib function call to execute
 * 
 * @note This macro contains a 'return' statement, so it can only be used in
 *       functions that return quadlib_result_t
 * 
 * @example
 * @code{.c}
 *     QUADLIB_CHECK(function_that_may_fail(args));
 * @endcode 
 */
#define QUADLIB_CHECK(func_call) do { \
    quadlib_result_t _res = (func_call); \
    if (_res != QUADLIB_SUCCESS) { \
        switch (_res) { \
            case QUADLIB_WARNING: \
                fprintf(stderr, "[QUADLIB] %s warning: Check mini_quadlib_logs.txt\n", #func_call); \
                break; \
            case QUADLIB_ERROR_INVALID_INPUT_NUMBER: \
                fprintf(stderr, "[QUADLIB] %s failed: Invalid input number\n", #func_call); \
                break; \
            case QUADLIB_ERROR_INVALID_INPUT_STRING: \
                fprintf(stderr, "[QUADLIB] %s failed: Invalid input string\n", #func_call); \
                break; \
            case QUADLIB_ERROR_SAFETY_VIOLATION: \
                fprintf(stderr, "[QUADLIB] %s failed: Safety violation detected\n", #func_call); \
                break; \
            default: \
                fprintf(stderr, "[QUADLIB] %s failed: Unknown error (%d)\n", #func_call, _res); \
                break; \
        } \
        assert((_res == QUADLIB_SUCCESS || _res == QUADLIB_WARNING) && "QUADLIB internal function failed: " #func_call); \
        return _res; \
    } \
} while(0)

// =============================================================================
// CONSTANTS
// =============================================================================

#define QUADLIB_GRAVITY 9.81f
#define QUADLIB_EPSILON 5e-7f

# define QUADLIB_PI	3.14159265f	    // pi
# define QUADLIB_PI_2 1.57079633f	// pi/2
# define QUADLIB_PI_4 0.78539816f   // pi/4

// =============================================================================
// HELPER MACROS
// =============================================================================

#define QUADLIB_DEG_TO_RAD(deg) ((float)((deg) / 180.0f * QUADLIB_PI))
#define QUADLIB_RAD_TO_DEG(rad) ((float)((rad) * 180.0f / QUADLIB_PI))
#define QUADLIB_CLAMP(value, min, max) ((value) < (min) ? (min) : (value) > (max) ? (max) : (value))

#ifdef __cplusplus
}
#endif

#endif // MINI_QUADLIB_H
