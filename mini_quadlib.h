/**
 * @file mini_quadlib.h
 * @brief Mini library for quadrotor control
 * 
 * This library provides essential functions for quadrotor control including:
 * - Geometric controller
 * - Coordinate conversions (quaternion, Euler angles, rotation matrices)
 * - Frame conversions (ENU, NED, etc.)
 */

#ifndef MINI_QUADLIB_H
#define MINI_QUADLIB_H

#include <math.h>

/* ============================================================================
 * Constants and Definitions
 * ============================================================================ */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Frame types for coordinate frame conversions */
typedef enum {
    FRAME_ENU,  /* East-North-Up */
    FRAME_NED,  /* North-East-Down */
    FRAME_NWU,  /* North-West-Up */
    FRAME_FLU,  /* Forward-Left-Up */
    FRAME_FRD   /* Forward-Right-Down */
} FrameType;

/* ============================================================================
 * Vector and Matrix Structures
 * ============================================================================ */

typedef struct {
    double x;
    double y;
    double z;
} Vector3;

typedef struct {
    double w;  /* scalar part */
    double x;
    double y;
    double z;
} Quaternion;

typedef struct {
    double roll;   /* rotation about x-axis (radians) */
    double pitch;  /* rotation about y-axis (radians) */
    double yaw;    /* rotation about z-axis (radians) */
} EulerAngles;

typedef struct {
    double m[3][3];  /* 3x3 rotation matrix */
} RotationMatrix;

/* ============================================================================
 * Quadrotor State
 * ============================================================================ */

typedef struct {
    Vector3 position;      /* position in world frame */
    Vector3 velocity;      /* velocity in world frame */
    Quaternion attitude;   /* orientation quaternion */
    Vector3 angular_vel;   /* angular velocity in body frame */
} QuadrotorState;

/* ============================================================================
 * Geometric Controller Parameters and Output
 * ============================================================================ */

typedef struct {
    double mass;           /* quadrotor mass (kg) */
    double gravity;        /* gravitational acceleration (m/s^2) */
    double kx;             /* position gain */
    double kv;             /* velocity gain */
    double kR;             /* attitude gain */
    double kOmega;         /* angular velocity gain */
} GeometricControllerGains;

typedef struct {
    double thrust;         /* total thrust (N) */
    Vector3 moment;        /* body moment (Nm) */
} ControlCommand;

/* ============================================================================
 * Quaternion Operations
 * ============================================================================ */

/**
 * @brief Create a quaternion from components
 */
Quaternion quaternion_create(double w, double x, double y, double z);

/**
 * @brief Normalize a quaternion
 */
Quaternion quaternion_normalize(Quaternion q);

/**
 * @brief Multiply two quaternions (q1 * q2)
 */
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2);

/**
 * @brief Compute quaternion conjugate
 */
Quaternion quaternion_conjugate(Quaternion q);

/**
 * @brief Compute quaternion inverse
 */
Quaternion quaternion_inverse(Quaternion q);

/**
 * @brief Convert quaternion to rotation matrix
 */
RotationMatrix quaternion_to_rotation_matrix(Quaternion q);

/**
 * @brief Convert quaternion to Euler angles (ZYX convention)
 */
EulerAngles quaternion_to_euler(Quaternion q);

/**
 * @brief Rotate a vector by a quaternion
 */
Vector3 quaternion_rotate_vector(Quaternion q, Vector3 v);

/* ============================================================================
 * Euler Angle Operations
 * ============================================================================ */

/**
 * @brief Create Euler angles from roll, pitch, yaw
 */
EulerAngles euler_create(double roll, double pitch, double yaw);

/**
 * @brief Convert Euler angles to quaternion (ZYX convention)
 */
Quaternion euler_to_quaternion(EulerAngles e);

/**
 * @brief Convert Euler angles to rotation matrix (ZYX convention)
 */
RotationMatrix euler_to_rotation_matrix(EulerAngles e);

/* ============================================================================
 * Rotation Matrix Operations
 * ============================================================================ */

/**
 * @brief Create identity rotation matrix
 */
RotationMatrix rotation_matrix_identity(void);

/**
 * @brief Convert rotation matrix to quaternion
 */
Quaternion rotation_matrix_to_quaternion(RotationMatrix R);

/**
 * @brief Convert rotation matrix to Euler angles (ZYX convention)
 */
EulerAngles rotation_matrix_to_euler(RotationMatrix R);

/**
 * @brief Multiply two rotation matrices (R1 * R2)
 */
RotationMatrix rotation_matrix_multiply(RotationMatrix R1, RotationMatrix R2);

/**
 * @brief Transpose a rotation matrix
 */
RotationMatrix rotation_matrix_transpose(RotationMatrix R);

/**
 * @brief Rotate a vector by a rotation matrix
 */
Vector3 rotation_matrix_rotate_vector(RotationMatrix R, Vector3 v);

/* ============================================================================
 * Vector Operations
 * ============================================================================ */

/**
 * @brief Create a vector from components
 */
Vector3 vector3_create(double x, double y, double z);

/**
 * @brief Add two vectors
 */
Vector3 vector3_add(Vector3 v1, Vector3 v2);

/**
 * @brief Subtract two vectors (v1 - v2)
 */
Vector3 vector3_subtract(Vector3 v1, Vector3 v2);

/**
 * @brief Multiply vector by scalar
 */
Vector3 vector3_scale(Vector3 v, double s);

/**
 * @brief Compute dot product
 */
double vector3_dot(Vector3 v1, Vector3 v2);

/**
 * @brief Compute cross product (v1 × v2)
 */
Vector3 vector3_cross(Vector3 v1, Vector3 v2);

/**
 * @brief Compute vector norm (magnitude)
 */
double vector3_norm(Vector3 v);

/**
 * @brief Normalize a vector
 */
Vector3 vector3_normalize(Vector3 v);

/* ============================================================================
 * Frame Conversion Functions
 * ============================================================================ */

/**
 * @brief Convert a vector from one frame to another
 * @param v Vector to convert
 * @param from_frame Source frame type
 * @param to_frame Destination frame type
 * @return Converted vector
 */
Vector3 convert_frame(Vector3 v, FrameType from_frame, FrameType to_frame);

/**
 * @brief Get rotation matrix for converting from one frame to another
 * @param from_frame Source frame type
 * @param to_frame Destination frame type
 * @return Rotation matrix for conversion
 */
RotationMatrix get_frame_conversion_matrix(FrameType from_frame, FrameType to_frame);

/* ============================================================================
 * Geometric Controller
 * ============================================================================ */

/**
 * @brief Initialize geometric controller gains with default values
 */
GeometricControllerGains geometric_controller_default_gains(void);

/**
 * @brief Compute control command using geometric controller
 * @param state Current quadrotor state
 * @param desired_pos Desired position
 * @param desired_vel Desired velocity
 * @param desired_acc Desired acceleration (feedforward)
 * @param desired_yaw Desired yaw angle (radians)
 * @param gains Controller gains
 * @return Control command (thrust and moment)
 */
ControlCommand geometric_controller_compute(
    QuadrotorState state,
    Vector3 desired_pos,
    Vector3 desired_vel,
    Vector3 desired_acc,
    double desired_yaw,
    GeometricControllerGains gains
);

/**
 * @brief Extract rotation matrix from quaternion attitude
 */
RotationMatrix get_rotation_from_state(QuadrotorState state);

#endif /* MINI_QUADLIB_H */
