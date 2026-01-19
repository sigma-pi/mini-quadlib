#include "../include/mini_quadlib.h"

// =============================================================================
// CORE CONTROLLER FUNCTIONS: GEOMETRIC CONTROLLER
// =============================================================================

static matrix3f_t unit_vec(vector3f_t q, vector3f_t q_dot, vector3f_t q_ddot)
{
    float q_norm = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z);
    float q_qdot = q.x * q_dot.x + q.y * q_dot.y + q.z * q_dot.z;
    float qdot_qdot = q_dot.x * q_dot.x + q_dot.y * q_dot.y + q_dot.z * q_dot.z;
    float q_qddot = q.x * q_ddot.x + q.y * q_ddot.y + q.z * q_ddot.z;

    vector3f_t u = {
        q.x / q_norm,
        q.y / q_norm,
        q.z / q_norm
    };

    vector3f_t u_dot = {
        q_dot.x / q_norm - q.x * q_qdot / powf(q_norm, 3),
        q_dot.y / q_norm - q.y * q_qdot / powf(q_norm, 3),
        q_dot.z / q_norm - q.z * q_qdot / powf(q_norm, 3)
    };

    vector3f_t u_ddot = {
        q_ddot.x / q_norm - q_dot.x / powf(q_norm, 3) * 2.0f * q_qdot - q.x / powf(q_norm, 3) * (qdot_qdot + q_qddot) + q.x * 3.0f / powf(q_norm, 5) * powf(q_qdot, 2),
        q_ddot.y / q_norm - q_dot.y / powf(q_norm, 3) * 2.0f * q_qdot - q.y / powf(q_norm, 3) * (qdot_qdot + q_qddot) + q.y * 3.0f / powf(q_norm, 5) * powf(q_qdot, 2),
        q_ddot.z / q_norm - q_dot.z / powf(q_norm, 3) * 2.0f * q_qdot - q.z / powf(q_norm, 3) * (qdot_qdot + q_qddot) + q.z * 3.0f / powf(q_norm, 5) * powf(q_qdot, 2)
    };

    matrix3f_t result = {
        .colx = u,
        .coly = u_dot,
        .colz = u_ddot
    };

    return result;
}

quadlib_result_t geometric_controller_TM_fullparam(control_4f_t* output_control_TM,
                                                    const state_t* current_state,
                                                    const setpoint_t* desired_state,
                                                    const geometric_params_t* ctrl_params,
                                                    const quadx_params_t* quad_params)
{
    matrix3f_t mat1_temp;
    matrix3f_t mat2_temp;
    matrix3f_t mat3_temp;
    vector3f_t vec1_temp;
    vector3f_t vec2_temp;
    vector3f_t vec3_temp;
    float scal1_temp;
    float scal2_temp;

    vector3f_t p_error;
    vector3f_t v_error;
    QUADLIB_CHECK(vector3_sub(&p_error, &current_state->pos, &desired_state->pos));
    QUADLIB_CHECK(vector3_sub(&v_error, &current_state->vel, &desired_state->vel));

    vector3f_t target_force;
    target_force.x = quad_params->mass * desired_state->acc.x
                   - ctrl_params->k_p.x * p_error.x
                   - ctrl_params->k_v.x * v_error.x;
    target_force.y = quad_params->mass * desired_state->acc.y
                   - ctrl_params->k_p.y * p_error.y
                   - ctrl_params->k_v.y * v_error.y;
    target_force.z = quad_params->mass * (desired_state->acc.z - QUADLIB_GRAVITY)
                   - ctrl_params->k_p.z * p_error.z
                   - ctrl_params->k_v.z * v_error.z;

    matrix3f_t R;
    QUADLIB_CHECK(quaternion_to_rotation_matrix(&R, &current_state->quat));

    float target_thrust;
    QUADLIB_CHECK(vector3_dot(&target_thrust, &target_force, &R.colz));
    target_thrust = -target_thrust;

    vector3f_t z_axis_desired = {-target_force.x,
                                 -target_force.y,
                                 -target_force.z};
    QUADLIB_CHECK(vector3_normalize(&z_axis_desired, &z_axis_desired));

    vector3f_t x_c_des = {cosf(desired_state->yaw),
                          sinf(desired_state->yaw),
                          0.0f};
    vector3f_t x_c_des_dot = {-sinf(desired_state->yaw) * desired_state->yaw_dot,
                              cosf(desired_state->yaw) * desired_state->yaw_dot,
                              0.0f};
    vector3f_t x_c_des_ddot = {-cosf(desired_state->yaw) * desired_state->yaw_dot * desired_state->yaw_dot
                               -sinf(desired_state->yaw) * desired_state->yaw_ddot,
                               -sinf(desired_state->yaw) * desired_state->yaw_dot * desired_state->yaw_dot
                               +cosf(desired_state->yaw) * desired_state->yaw_ddot,
                               0.0f};
    
    vector3f_t y_axis_desired;
    QUADLIB_CHECK(vector3_cross(&y_axis_desired, &z_axis_desired, &x_c_des));
    QUADLIB_CHECK(vector3_normalize(&y_axis_desired, &y_axis_desired));

    vector3f_t x_axis_desired;
    QUADLIB_CHECK(vector3_cross(&x_axis_desired, &y_axis_desired, &z_axis_desired));
    QUADLIB_CHECK(vector3_normalize(&x_axis_desired, &x_axis_desired));

    matrix3f_t Rdes = {
        .colx = x_axis_desired,
        .coly = y_axis_desired,
        .colz = z_axis_desired
    };

    matrix3f_t Rdes_T;
    matrix3f_t R_T;
    matrix3f_t Rdes_T_R;
    matrix3f_t R_T_Rdes;
    QUADLIB_CHECK(rotation_matrix_transpose(&Rdes_T, &Rdes));
    QUADLIB_CHECK(rotation_matrix_transpose(&R_T, &R));
    QUADLIB_CHECK(matrix3_dot(&Rdes_T_R, &Rdes_T, &R));
    QUADLIB_CHECK(matrix3_dot(&R_T_Rdes, &R_T, &Rdes));
    QUADLIB_CHECK(matrix3_sub(&mat1_temp, &Rdes_T_R, &R_T_Rdes));
    matrix3f_t eRM = {
        .colx = {0.5f * mat1_temp.colx.x, 0.5f * mat1_temp.colx.y, 0.5f * mat1_temp.colx.z},
        .coly = {0.5f * mat1_temp.coly.x, 0.5f * mat1_temp.coly.y, 0.5f * mat1_temp.coly.z},
        .colz = {0.5f * mat1_temp.colz.x, 0.5f * mat1_temp.colz.y, 0.5f * mat1_temp.colz.z}
    };
    vector3f_t eR;
    QUADLIB_CHECK(matrix3_vee(&eR, &eRM));

    vector3f_t a_error = {
        -R.colz.x * target_thrust / quad_params->mass - desired_state->acc.x,
        -R.colz.y * target_thrust / quad_params->mass - desired_state->acc.y,
        -R.colz.z * target_thrust / quad_params->mass - desired_state->acc.z + QUADLIB_GRAVITY
    };
    
    vector3f_t target_force_dot = {
        -ctrl_params->k_p.x * v_error.x - ctrl_params->k_v.x * a_error.x + quad_params->mass * desired_state->jerk.x,
        -ctrl_params->k_p.y * v_error.y - ctrl_params->k_v.y * a_error.y + quad_params->mass * desired_state->jerk.y,
        -ctrl_params->k_p.z * v_error.z - ctrl_params->k_v.z * a_error.z + quad_params->mass * desired_state->jerk.z
    };

    matrix3f_t omega_hat;
    QUADLIB_CHECK(vector3_hat(&omega_hat, &current_state->omega));
    QUADLIB_CHECK(matrix3_dot(&mat1_temp, &R, &omega_hat));
    vector3f_t b3_dot = mat1_temp.colz;

    QUADLIB_CHECK(vector3_dot(&scal1_temp, &target_force_dot, &R.colz));
    QUADLIB_CHECK(vector3_dot(&scal2_temp, &target_force, &b3_dot));
    float target_thrust_dot = -scal1_temp - scal2_temp;

    vector3f_t j_error = {
        -R.colz.x * target_thrust_dot / quad_params->mass - b3_dot.x * target_thrust / quad_params->mass - desired_state->jerk.x,
        -R.colz.y * target_thrust_dot / quad_params->mass - b3_dot.y * target_thrust / quad_params->mass - desired_state->jerk.y,
        -R.colz.z * target_thrust_dot / quad_params->mass - b3_dot.z * target_thrust / quad_params->mass - desired_state->jerk.z
    };

    vector3f_t target_force_ddot = {
        -ctrl_params->k_p.x * a_error.x - ctrl_params->k_v.x * j_error.x + quad_params->mass * desired_state->snap.x,
        -ctrl_params->k_p.y * a_error.y - ctrl_params->k_v.y * j_error.y + quad_params->mass * desired_state->snap.y,
        -ctrl_params->k_p.z * a_error.z - ctrl_params->k_v.z * j_error.z + quad_params->mass * desired_state->snap.z
    };

    matrix3f_t b3cCollection = unit_vec((vector3f_t){-target_force.x, -target_force.y, -target_force.z},
                                        (vector3f_t){-target_force_dot.x, -target_force_dot.y, -target_force_dot.z},
                                        (vector3f_t){-target_force_ddot.x, -target_force_ddot.y, -target_force_ddot.z});      
    vector3f_t b3c = b3cCollection.colx;
    vector3f_t b3c_dot = b3cCollection.coly;
    vector3f_t b3c_ddot = b3cCollection.colz;

    QUADLIB_CHECK(vector3_cross(&vec1_temp, &x_c_des, &b3c));
    vector3f_t A2 = {
        -vec1_temp.x,
        -vec1_temp.y,
        -vec1_temp.z
    };
    QUADLIB_CHECK(vector3_cross(&vec1_temp, &x_c_des_dot, &b3c));
    QUADLIB_CHECK(vector3_cross(&vec2_temp, &x_c_des, &b3c_dot));
    vector3f_t A2_dot = {
        -(vec1_temp.x + vec2_temp.x),
        -(vec1_temp.y + vec2_temp.y),
        -(vec1_temp.z + vec2_temp.z)
    };
    QUADLIB_CHECK(vector3_cross(&vec1_temp, &x_c_des_ddot, &b3c));
    QUADLIB_CHECK(vector3_cross(&vec2_temp, &x_c_des_dot, &b3c_dot));
    QUADLIB_CHECK(vector3_cross(&vec3_temp, &x_c_des, &b3c_ddot));
    vector3f_t A2_ddot = {
        -(vec1_temp.x + 2.0f * vec2_temp.x + vec3_temp.x),
        -(vec1_temp.y + 2.0f * vec2_temp.y + vec3_temp.y),
        -(vec1_temp.z + 2.0f * vec2_temp.z + vec3_temp.z)
    };

    matrix3f_t b2cCollection = unit_vec(A2, A2_dot, A2_ddot);
    vector3f_t b2c = b2cCollection.colx;
    vector3f_t b2c_dot = b2cCollection.coly;
    vector3f_t b2c_ddot = b2cCollection.colz;

    QUADLIB_CHECK(vector3_cross(&vec1_temp, &b2c_dot, &b3c));
    QUADLIB_CHECK(vector3_cross(&vec2_temp, &b2c, &b3c_dot));
    vector3f_t b1c_dot = {
        vec1_temp.x + vec2_temp.x,
        vec1_temp.y + vec2_temp.y,
        vec1_temp.z + vec2_temp.z
    };

    QUADLIB_CHECK(vector3_cross(&vec1_temp, &b2c_ddot, &b3c));
    QUADLIB_CHECK(vector3_cross(&vec2_temp, &b2c_dot, &b3c_dot));
    QUADLIB_CHECK(vector3_cross(&vec3_temp, &b2c, &b3c_ddot));
    vector3f_t b1c_ddot = {
        vec1_temp.x + 2.0f * vec2_temp.x + vec3_temp.x,
        vec1_temp.y + 2.0f * vec2_temp.y + vec3_temp.y,
        vec1_temp.z + 2.0f * vec2_temp.z + vec3_temp.z
    };

    matrix3f_t Rd_dot = {
        .colx = b1c_dot,
        .coly = b2c_dot,
        .colz = b3c_dot
    };
    matrix3f_t Rd_ddot = {
        .colx = b1c_ddot,
        .coly = b2c_ddot,
        .colz = b3c_ddot
    };

    vector3f_t omegad;
    QUADLIB_CHECK(matrix3_dot(&mat1_temp, &Rdes_T, &Rd_dot));
    QUADLIB_CHECK(matrix3_vee(&omegad, &mat1_temp));
    vector3f_t omegad_dot;
    QUADLIB_CHECK(matrix3_dot(&mat1_temp, &Rdes_T, &Rd_ddot));
    QUADLIB_CHECK(matrix3_dot(&mat2_temp, &omega_hat, &omega_hat));
    QUADLIB_CHECK(matrix3_sub(&mat3_temp, &mat1_temp, &mat2_temp));
    QUADLIB_CHECK(matrix3_vee(&omegad_dot, &mat3_temp));

    QUADLIB_CHECK(matrix3_dot(&mat1_temp, &R_T, &Rdes));
    QUADLIB_CHECK(matrix3_dotv(&vec1_temp, &mat1_temp, &omegad));
    vector3f_t ew = {
        current_state->omega.x - vec1_temp.x,
        current_state->omega.y - vec1_temp.y,
        current_state->omega.z - vec1_temp.z
    };

    vector3f_t M = {
        -ctrl_params->k_R.x * eR.x - ctrl_params->k_O.x * ew.x,
        -ctrl_params->k_R.y * eR.y - ctrl_params->k_O.y * ew.y,
        -ctrl_params->k_R.z * eR.z - ctrl_params->k_O.z * ew.z
    };
    // M = M - J * (hatOperator(Omega) * R.transposed() * Rdes * Omegad - R.transposed() * Rdes * Omegad_dot);
    QUADLIB_CHECK(matrix3_dotv(&vec2_temp, &mat1_temp, &omegad_dot));
    // M = M - J * (omega_hat * vec1_temp - vec2_temp);
    QUADLIB_CHECK(vector3_cross(&vec3_temp, &current_state->omega, &vec1_temp));
    // M = M - J * (vec3_temp - vec2_temp);
    QUADLIB_CHECK(vector3_sub(&vec1_temp, &vec3_temp, &vec2_temp));
    // M = M - J * vec1_temp;
    M.x = M.x - quad_params->inertia.x * vec1_temp.x;
    M.y = M.y - quad_params->inertia.y * vec1_temp.y;
    M.z = M.z - quad_params->inertia.z * vec1_temp.z;
    // momentAdd = Omega % (J * Omega);
    vec1_temp.x = quad_params->inertia.x * current_state->omega.x;
    vec1_temp.y = quad_params->inertia.y * current_state->omega.y;
    vec1_temp.z = quad_params->inertia.z * current_state->omega.z;
    QUADLIB_CHECK(vector3_cross(&vec2_temp, &current_state->omega, &vec1_temp));
    // M = M + momentAdd;
    M.x = M.x + vec2_temp.x;
    M.y = M.y + vec2_temp.y;
    M.z = M.z + vec2_temp.z;

    // Thrust and Moment output
    output_control_TM->u1 = target_thrust;
    output_control_TM->u2 = M.x;
    output_control_TM->u3 = M.y;
    output_control_TM->u4 = M.z;

    return QUADLIB_SUCCESS;
}