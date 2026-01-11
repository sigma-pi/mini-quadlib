/**
 * @file mini_quadlib.c
 * @brief Implementation of mini quadrotor control library
 */

#include "mini_quadlib.h"
#include <math.h>
#include <string.h>

/* ============================================================================
 * Quaternion Operations
 * ============================================================================ */

Quaternion quaternion_create(double w, double x, double y, double z) {
    Quaternion q;
    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;
    return q;
}

Quaternion quaternion_normalize(Quaternion q) {
    double norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm < 1e-10) {
        /* Return identity quaternion if norm is too small */
        return quaternion_create(1.0, 0.0, 0.0, 0.0);
    }
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    return q;
}

Quaternion quaternion_multiply(Quaternion q1, Quaternion q2) {
    Quaternion result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

Quaternion quaternion_conjugate(Quaternion q) {
    Quaternion result;
    result.w = q.w;
    result.x = -q.x;
    result.y = -q.y;
    result.z = -q.z;
    return result;
}

Quaternion quaternion_inverse(Quaternion q) {
    double norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (norm_sq < 1e-10) {
        /* Return identity quaternion if norm is too small */
        return quaternion_create(1.0, 0.0, 0.0, 0.0);
    }
    Quaternion conj = quaternion_conjugate(q);
    conj.w /= norm_sq;
    conj.x /= norm_sq;
    conj.y /= norm_sq;
    conj.z /= norm_sq;
    return conj;
}

RotationMatrix quaternion_to_rotation_matrix(Quaternion q) {
    RotationMatrix R;
    
    /* Normalize quaternion first */
    q = quaternion_normalize(q);
    
    double w = q.w, x = q.x, y = q.y, z = q.z;
    double w2 = w * w, x2 = x * x, y2 = y * y, z2 = z * z;
    
    R.m[0][0] = w2 + x2 - y2 - z2;
    R.m[0][1] = 2.0 * (x * y - w * z);
    R.m[0][2] = 2.0 * (x * z + w * y);
    
    R.m[1][0] = 2.0 * (x * y + w * z);
    R.m[1][1] = w2 - x2 + y2 - z2;
    R.m[1][2] = 2.0 * (y * z - w * x);
    
    R.m[2][0] = 2.0 * (x * z - w * y);
    R.m[2][1] = 2.0 * (y * z + w * x);
    R.m[2][2] = w2 - x2 - y2 + z2;
    
    return R;
}

EulerAngles quaternion_to_euler(Quaternion q) {
    EulerAngles e;
    
    /* Normalize quaternion first */
    q = quaternion_normalize(q);
    
    double w = q.w, x = q.x, y = q.y, z = q.z;
    
    /* Roll (x-axis rotation) */
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    e.roll = atan2(sinr_cosp, cosr_cosp);
    
    /* Pitch (y-axis rotation) */
    double sinp = 2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1.0) {
        e.pitch = copysign(M_PI / 2.0, sinp); /* Use ±90 degrees if out of range */
    } else {
        e.pitch = asin(sinp);
    }
    
    /* Yaw (z-axis rotation) */
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    e.yaw = atan2(siny_cosp, cosy_cosp);
    
    return e;
}

Vector3 quaternion_rotate_vector(Quaternion q, Vector3 v) {
    /* Rotate vector using quaternion: v' = q * v * q^(-1) */
    Quaternion v_quat = quaternion_create(0.0, v.x, v.y, v.z);
    Quaternion q_inv = quaternion_conjugate(q); /* For unit quaternion, conjugate = inverse */
    
    Quaternion temp = quaternion_multiply(q, v_quat);
    Quaternion result_quat = quaternion_multiply(temp, q_inv);
    
    Vector3 result;
    result.x = result_quat.x;
    result.y = result_quat.y;
    result.z = result_quat.z;
    
    return result;
}

/* ============================================================================
 * Euler Angle Operations
 * ============================================================================ */

EulerAngles euler_create(double roll, double pitch, double yaw) {
    EulerAngles e;
    e.roll = roll;
    e.pitch = pitch;
    e.yaw = yaw;
    return e;
}

Quaternion euler_to_quaternion(EulerAngles e) {
    /* Convert Euler angles to quaternion using ZYX convention */
    double cy = cos(e.yaw * 0.5);
    double sy = sin(e.yaw * 0.5);
    double cp = cos(e.pitch * 0.5);
    double sp = sin(e.pitch * 0.5);
    double cr = cos(e.roll * 0.5);
    double sr = sin(e.roll * 0.5);
    
    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return quaternion_normalize(q);
}

RotationMatrix euler_to_rotation_matrix(EulerAngles e) {
    RotationMatrix R;
    
    double cr = cos(e.roll);
    double sr = sin(e.roll);
    double cp = cos(e.pitch);
    double sp = sin(e.pitch);
    double cy = cos(e.yaw);
    double sy = sin(e.yaw);
    
    /* ZYX Euler angle convention */
    R.m[0][0] = cy * cp;
    R.m[0][1] = cy * sp * sr - sy * cr;
    R.m[0][2] = cy * sp * cr + sy * sr;
    
    R.m[1][0] = sy * cp;
    R.m[1][1] = sy * sp * sr + cy * cr;
    R.m[1][2] = sy * sp * cr - cy * sr;
    
    R.m[2][0] = -sp;
    R.m[2][1] = cp * sr;
    R.m[2][2] = cp * cr;
    
    return R;
}

/* ============================================================================
 * Rotation Matrix Operations
 * ============================================================================ */

RotationMatrix rotation_matrix_identity(void) {
    RotationMatrix R;
    memset(&R, 0, sizeof(RotationMatrix));
    R.m[0][0] = 1.0;
    R.m[1][1] = 1.0;
    R.m[2][2] = 1.0;
    return R;
}

Quaternion rotation_matrix_to_quaternion(RotationMatrix R) {
    Quaternion q;
    
    double trace = R.m[0][0] + R.m[1][1] + R.m[2][2];
    
    if (trace > 0.0) {
        double s = 0.5 / sqrt(trace + 1.0);
        q.w = 0.25 / s;
        q.x = (R.m[2][1] - R.m[1][2]) * s;
        q.y = (R.m[0][2] - R.m[2][0]) * s;
        q.z = (R.m[1][0] - R.m[0][1]) * s;
    } else if (R.m[0][0] > R.m[1][1] && R.m[0][0] > R.m[2][2]) {
        double s = 2.0 * sqrt(1.0 + R.m[0][0] - R.m[1][1] - R.m[2][2]);
        q.w = (R.m[2][1] - R.m[1][2]) / s;
        q.x = 0.25 * s;
        q.y = (R.m[0][1] + R.m[1][0]) / s;
        q.z = (R.m[0][2] + R.m[2][0]) / s;
    } else if (R.m[1][1] > R.m[2][2]) {
        double s = 2.0 * sqrt(1.0 + R.m[1][1] - R.m[0][0] - R.m[2][2]);
        q.w = (R.m[0][2] - R.m[2][0]) / s;
        q.x = (R.m[0][1] + R.m[1][0]) / s;
        q.y = 0.25 * s;
        q.z = (R.m[1][2] + R.m[2][1]) / s;
    } else {
        double s = 2.0 * sqrt(1.0 + R.m[2][2] - R.m[0][0] - R.m[1][1]);
        q.w = (R.m[1][0] - R.m[0][1]) / s;
        q.x = (R.m[0][2] + R.m[2][0]) / s;
        q.y = (R.m[1][2] + R.m[2][1]) / s;
        q.z = 0.25 * s;
    }
    
    return quaternion_normalize(q);
}

EulerAngles rotation_matrix_to_euler(RotationMatrix R) {
    EulerAngles e;
    
    /* ZYX convention */
    /* Clamp R.m[2][0] to [-1, 1] to handle numerical errors */
    double sin_pitch = -R.m[2][0];
    if (sin_pitch > 1.0) sin_pitch = 1.0;
    if (sin_pitch < -1.0) sin_pitch = -1.0;
    e.pitch = asin(sin_pitch);
    
    if (fabs(cos(e.pitch)) > 1e-6) {
        e.roll = atan2(R.m[2][1], R.m[2][2]);
        e.yaw = atan2(R.m[1][0], R.m[0][0]);
    } else {
        /* Gimbal lock case */
        e.roll = 0.0;
        e.yaw = atan2(-R.m[0][1], R.m[1][1]);
    }
    
    return e;
}

RotationMatrix rotation_matrix_multiply(RotationMatrix R1, RotationMatrix R2) {
    RotationMatrix result;
    int i, j, k;
    
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            result.m[i][j] = 0.0;
            for (k = 0; k < 3; k++) {
                result.m[i][j] += R1.m[i][k] * R2.m[k][j];
            }
        }
    }
    
    return result;
}

RotationMatrix rotation_matrix_transpose(RotationMatrix R) {
    RotationMatrix result;
    int i, j;
    
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            result.m[i][j] = R.m[j][i];
        }
    }
    
    return result;
}

Vector3 rotation_matrix_rotate_vector(RotationMatrix R, Vector3 v) {
    Vector3 result;
    result.x = R.m[0][0] * v.x + R.m[0][1] * v.y + R.m[0][2] * v.z;
    result.y = R.m[1][0] * v.x + R.m[1][1] * v.y + R.m[1][2] * v.z;
    result.z = R.m[2][0] * v.x + R.m[2][1] * v.y + R.m[2][2] * v.z;
    return result;
}

/* ============================================================================
 * Vector Operations
 * ============================================================================ */

Vector3 vector3_create(double x, double y, double z) {
    Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

Vector3 vector3_add(Vector3 v1, Vector3 v2) {
    Vector3 result;
    result.x = v1.x + v2.x;
    result.y = v1.y + v2.y;
    result.z = v1.z + v2.z;
    return result;
}

Vector3 vector3_subtract(Vector3 v1, Vector3 v2) {
    Vector3 result;
    result.x = v1.x - v2.x;
    result.y = v1.y - v2.y;
    result.z = v1.z - v2.z;
    return result;
}

Vector3 vector3_scale(Vector3 v, double s) {
    Vector3 result;
    result.x = v.x * s;
    result.y = v.y * s;
    result.z = v.z * s;
    return result;
}

double vector3_dot(Vector3 v1, Vector3 v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

Vector3 vector3_cross(Vector3 v1, Vector3 v2) {
    Vector3 result;
    result.x = v1.y * v2.z - v1.z * v2.y;
    result.y = v1.z * v2.x - v1.x * v2.z;
    result.z = v1.x * v2.y - v1.y * v2.x;
    return result;
}

double vector3_norm(Vector3 v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vector3 vector3_normalize(Vector3 v) {
    double norm = vector3_norm(v);
    if (norm < 1e-10) {
        /* Return unit z-axis as default for consistency with geometric controller */
        return vector3_create(0.0, 0.0, 1.0);
    }
    return vector3_scale(v, 1.0 / norm);
}

/* ============================================================================
 * Frame Conversion Functions
 * ============================================================================ */

RotationMatrix get_frame_conversion_matrix(FrameType from_frame, FrameType to_frame) {
    RotationMatrix R = rotation_matrix_identity();
    
    /* If same frame, return identity */
    if (from_frame == to_frame) {
        return R;
    }
    
    /* Define conversion matrices for each frame pair */
    /* The approach: define each frame relative to ENU, then compose transformations */
    
    /* First, convert from source frame to ENU (if not already ENU) */
    RotationMatrix R_from_to_enu = rotation_matrix_identity();
    
    switch (from_frame) {
        case FRAME_ENU:
            /* Already in ENU, identity */
            break;
        case FRAME_NED:
            /* NED to ENU: x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned */
            R_from_to_enu.m[0][0] = 0.0; R_from_to_enu.m[0][1] = 1.0; R_from_to_enu.m[0][2] = 0.0;
            R_from_to_enu.m[1][0] = 1.0; R_from_to_enu.m[1][1] = 0.0; R_from_to_enu.m[1][2] = 0.0;
            R_from_to_enu.m[2][0] = 0.0; R_from_to_enu.m[2][1] = 0.0; R_from_to_enu.m[2][2] = -1.0;
            break;
        case FRAME_NWU:
            /* NWU to ENU: x_enu = -y_nwu, y_enu = x_nwu, z_enu = z_nwu */
            R_from_to_enu.m[0][0] = 0.0; R_from_to_enu.m[0][1] = -1.0; R_from_to_enu.m[0][2] = 0.0;
            R_from_to_enu.m[1][0] = 1.0; R_from_to_enu.m[1][1] = 0.0; R_from_to_enu.m[1][2] = 0.0;
            R_from_to_enu.m[2][0] = 0.0; R_from_to_enu.m[2][1] = 0.0; R_from_to_enu.m[2][2] = 1.0;
            break;
        case FRAME_FLU:
            /* FLU (body-fixed, Forward-Left-Up) to ENU: depends on vehicle orientation */
            /* NOTE: FLU is body-fixed and requires attitude information for proper conversion.
             * This assumes the body is aligned with ENU (zero attitude) as a placeholder.
             * For proper FLU conversions, use the rotation matrix from the vehicle's attitude. */
            break;
        case FRAME_FRD:
            /* FRD to ENU: x_enu = x_frd, y_enu = -y_frd, z_enu = -z_frd */
            R_from_to_enu.m[0][0] = 1.0; R_from_to_enu.m[0][1] = 0.0; R_from_to_enu.m[0][2] = 0.0;
            R_from_to_enu.m[1][0] = 0.0; R_from_to_enu.m[1][1] = -1.0; R_from_to_enu.m[1][2] = 0.0;
            R_from_to_enu.m[2][0] = 0.0; R_from_to_enu.m[2][1] = 0.0; R_from_to_enu.m[2][2] = -1.0;
            break;
    }
    
    /* Then, convert from ENU to destination frame */
    RotationMatrix R_enu_to_target = rotation_matrix_identity();
    
    switch (to_frame) {
        case FRAME_ENU:
            /* Already in ENU, identity */
            break;
        case FRAME_NED:
            /* ENU to NED: inverse of NED to ENU */
            R_enu_to_target.m[0][0] = 0.0; R_enu_to_target.m[0][1] = 1.0; R_enu_to_target.m[0][2] = 0.0;
            R_enu_to_target.m[1][0] = 1.0; R_enu_to_target.m[1][1] = 0.0; R_enu_to_target.m[1][2] = 0.0;
            R_enu_to_target.m[2][0] = 0.0; R_enu_to_target.m[2][1] = 0.0; R_enu_to_target.m[2][2] = -1.0;
            break;
        case FRAME_NWU:
            /* ENU to NWU: inverse of NWU to ENU */
            R_enu_to_target.m[0][0] = 0.0; R_enu_to_target.m[0][1] = 1.0; R_enu_to_target.m[0][2] = 0.0;
            R_enu_to_target.m[1][0] = -1.0; R_enu_to_target.m[1][1] = 0.0; R_enu_to_target.m[1][2] = 0.0;
            R_enu_to_target.m[2][0] = 0.0; R_enu_to_target.m[2][1] = 0.0; R_enu_to_target.m[2][2] = 1.0;
            break;
        case FRAME_FLU:
            /* ENU to FLU (body-fixed, Forward-Left-Up): depends on vehicle orientation */
            /* NOTE: FLU is body-fixed and requires attitude information for proper conversion.
             * This assumes the body is aligned with ENU (zero attitude) as a placeholder.
             * For proper FLU conversions, use the rotation matrix from the vehicle's attitude. */
            break;
        case FRAME_FRD:
            /* ENU to FRD: inverse of FRD to ENU */
            R_enu_to_target.m[0][0] = 1.0; R_enu_to_target.m[0][1] = 0.0; R_enu_to_target.m[0][2] = 0.0;
            R_enu_to_target.m[1][0] = 0.0; R_enu_to_target.m[1][1] = -1.0; R_enu_to_target.m[1][2] = 0.0;
            R_enu_to_target.m[2][0] = 0.0; R_enu_to_target.m[2][1] = 0.0; R_enu_to_target.m[2][2] = -1.0;
            break;
    }
    
    /* Compose the two transformations */
    R = rotation_matrix_multiply(R_enu_to_target, R_from_to_enu);
    
    return R;
}

Vector3 convert_frame(Vector3 v, FrameType from_frame, FrameType to_frame) {
    RotationMatrix R = get_frame_conversion_matrix(from_frame, to_frame);
    return rotation_matrix_rotate_vector(R, v);
}

/* ============================================================================
 * Geometric Controller
 * ============================================================================ */

GeometricControllerGains geometric_controller_default_gains(void) {
    GeometricControllerGains gains;
    gains.mass = 1.0;         /* 1 kg */
    gains.gravity = 9.81;     /* m/s^2 */
    gains.kx = 5.0;           /* position gain */
    gains.kv = 3.0;           /* velocity gain */
    gains.kR = 8.0;           /* attitude gain */
    gains.kOmega = 1.5;       /* angular velocity gain */
    return gains;
}

RotationMatrix get_rotation_from_state(QuadrotorState state) {
    return quaternion_to_rotation_matrix(state.attitude);
}

ControlCommand geometric_controller_compute(
    QuadrotorState state,
    Vector3 desired_pos,
    Vector3 desired_vel,
    Vector3 desired_acc,
    double desired_yaw,
    GeometricControllerGains gains
) {
    ControlCommand cmd;
    
    /* Position error */
    Vector3 pos_error = vector3_subtract(desired_pos, state.position);
    
    /* Velocity error */
    Vector3 vel_error = vector3_subtract(desired_vel, state.velocity);
    
    /* Desired acceleration with PD control */
    Vector3 acc_des = desired_acc;
    acc_des = vector3_add(acc_des, vector3_scale(pos_error, gains.kx));
    acc_des = vector3_add(acc_des, vector3_scale(vel_error, gains.kv));
    
    /* Gravity compensation */
    Vector3 gravity_vec = vector3_create(0.0, 0.0, gains.gravity);
    acc_des = vector3_add(acc_des, gravity_vec);
    
    /* Current rotation matrix */
    RotationMatrix R = get_rotation_from_state(state);
    
    /* z-axis of current rotation (thrust direction) */
    Vector3 z_body = vector3_create(R.m[0][2], R.m[1][2], R.m[2][2]);
    
    /* Thrust magnitude */
    cmd.thrust = gains.mass * vector3_dot(acc_des, z_body);
    
    /* Desired z-axis (normalized acceleration direction) */
    Vector3 z_des = vector3_normalize(acc_des);
    
    /* Desired x-axis based on desired yaw */
    Vector3 x_c = vector3_create(cos(desired_yaw), sin(desired_yaw), 0.0);
    
    /* Check if z_des and x_c are nearly parallel/anti-parallel */
    Vector3 y_des_temp = vector3_cross(z_des, x_c);
    if (vector3_norm(y_des_temp) < 1e-6) {
        /* If parallel, use alternative reference vector */
        x_c = vector3_create(0.0, 1.0, 0.0);  /* Use north as backup */
        y_des_temp = vector3_cross(z_des, x_c);
    }
    
    Vector3 y_des = vector3_normalize(y_des_temp);
    Vector3 x_des = vector3_cross(y_des, z_des);
    
    /* Desired rotation matrix */
    RotationMatrix R_des;
    R_des.m[0][0] = x_des.x; R_des.m[0][1] = y_des.x; R_des.m[0][2] = z_des.x;
    R_des.m[1][0] = x_des.y; R_des.m[1][1] = y_des.y; R_des.m[1][2] = z_des.y;
    R_des.m[2][0] = x_des.z; R_des.m[2][1] = y_des.z; R_des.m[2][2] = z_des.z;
    
    /* Rotation error: R_des^T * R - R^T * R_des (skew-symmetric part) */
    RotationMatrix R_des_T = rotation_matrix_transpose(R_des);
    RotationMatrix R_T = rotation_matrix_transpose(R);
    
    RotationMatrix eR_matrix1 = rotation_matrix_multiply(R_des_T, R);
    RotationMatrix eR_matrix2 = rotation_matrix_multiply(R_T, R_des);
    
    /* Extract rotation error vector from skew-symmetric matrix (eR_matrix1 - eR_matrix2) */
    /* The skew-symmetric matrix has the form: [0, -z, y; z, 0, -x; -y, x, 0] */
    Vector3 eR;
    eR.x = 0.5 * ((eR_matrix1.m[2][1] - eR_matrix1.m[1][2]) - (eR_matrix2.m[2][1] - eR_matrix2.m[1][2]));
    eR.y = 0.5 * ((eR_matrix1.m[0][2] - eR_matrix1.m[2][0]) - (eR_matrix2.m[0][2] - eR_matrix2.m[2][0]));
    eR.z = 0.5 * ((eR_matrix1.m[1][0] - eR_matrix1.m[0][1]) - (eR_matrix2.m[1][0] - eR_matrix2.m[0][1]));
    
    /* Angular velocity error (assuming zero desired angular velocity) */
    Vector3 eOmega = state.angular_vel;
    
    /* Moment command */
    cmd.moment = vector3_subtract(
        vector3_scale(eR, -gains.kR),
        vector3_scale(eOmega, gains.kOmega)
    );
    
    return cmd;
}
