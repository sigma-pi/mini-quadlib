#include "../include/mini_quadlib.h"

// =============================================================================
// CORE CONTROLLER FUNCTIONS: L1 ADAPTIVE CONTROL
// =============================================================================

quadlib_result_t l1_adaptive_control_fullparam(l1_state_t* previous,
                                               l1_state_t* current,
                                               const float dt,
                                               const l1_params_t* l1_params,
                                               const quadx_params_t* quad_params)
{
    vector3f_t vec1_temp;
    vector3f_t vec2_temp;
    vector3f_t vec3_temp;
    float scal1_temp;

    float m_inv = 1.0f / quad_params->mass;
    vector3f_t J_inv = (vector3f_t){
        1.0f / quad_params->inertia.x,
        1.0f / quad_params->inertia.y,
        1.0f / quad_params->inertia.z
    };

    vector3f_t vpred_error_prev = (vector3f_t){
        previous->vel_hat.x - previous->vel.x,
        previous->vel_hat.y - previous->vel.y,
        previous->vel_hat.z - previous->vel.z
    };
    vector3f_t Wpred_error_prev = (vector3f_t){
        previous->omega_hat.x - previous->omega.x,
        previous->omega_hat.y - previous->omega.y,
        previous->omega_hat.z - previous->omega.z
    };

    matrix3f_t R_prev;
    QUADLIB_CHECK(quaternion_to_rotation_matrix(&R_prev, &previous->quat));

    /*
    v_hat = v_hat_prev + (e3 * GRAVITY_MAGNITUDE 
                        - R_prev.colz() * (u_b_prev[0] + u_ad_prev[0] + sigma_m_hat_prev[0]) * massInverse 
                        + R_prev.colx() * sigma_um_hat_prev[0] * massInverse 
                        + R_prev.coly() * sigma_um_hat_prev[1] * massInverse 
                        + vpred_error_prev * As_v) * dt;
    tempVec = {u_b_prev[1] + u_ad_prev[1] + sigma_m_hat_prev[1], 
               u_b_prev[2] + u_ad_prev[2] + sigma_m_hat_prev[2], 
               u_b_prev[3] + u_ad_prev[3] + sigma_m_hat_prev[3]};
    omega_hat = omega_hat_prev + (-Jinv * (omega_prev % (J * omega_prev))
                                 + Jinv * tempVec + omegapred_error_prev * As_omega) * dt;
    */

    scal1_temp = previous->ub.u1 + previous->uad.u1 + previous->sigma_f_hat.z;
    vec1_temp = (vector3f_t){
        -R_prev.colz.x * scal1_temp * m_inv + R_prev.colx.x * previous->sigma_f_hat.x * m_inv + R_prev.coly.x * previous->sigma_f_hat.y * m_inv + vpred_error_prev.x * l1_params->As_v,
        -R_prev.colz.y * scal1_temp * m_inv + R_prev.colx.y * previous->sigma_f_hat.x * m_inv + R_prev.coly.y * previous->sigma_f_hat.y * m_inv + vpred_error_prev.y * l1_params->As_v,
        -R_prev.colz.z * scal1_temp * m_inv + R_prev.colx.z * previous->sigma_f_hat.x * m_inv + R_prev.coly.z * previous->sigma_f_hat.y * m_inv + vpred_error_prev.z * l1_params->As_v + QUADLIB_GRAVITY
    };
    current->vel_hat.x = previous->vel_hat.x + vec1_temp.x * dt;
    current->vel_hat.y = previous->vel_hat.y + vec1_temp.y * dt;
    current->vel_hat.z = previous->vel_hat.z + vec1_temp.z * dt;

    vec1_temp = (vector3f_t){
        quad_params->inertia.x * previous->omega.x,
        quad_params->inertia.y * previous->omega.y,
        quad_params->inertia.z * previous->omega.z
    };
    QUADLIB_CHECK(vector3_cross(&vec2_temp, &previous->omega, &vec1_temp));
    vec1_temp = (vector3f_t){
        previous->ub.u2 + previous->uad.u2 + previous->sigma_M_hat.x,
        previous->ub.u3 + previous->uad.u3 + previous->sigma_M_hat.y,
        previous->ub.u4 + previous->uad.u4 + previous->sigma_M_hat.z
    };
    vec3_temp = (vector3f_t){
        -J_inv.x * vec2_temp.x + J_inv.x * vec1_temp.x + Wpred_error_prev.x * l1_params->As_W,
        -J_inv.y * vec2_temp.y + J_inv.y * vec1_temp.y + Wpred_error_prev.y * l1_params->As_W,
        -J_inv.z * vec2_temp.z + J_inv.z * vec1_temp.z + Wpred_error_prev.z * l1_params->As_W
    };
    current->omega_hat.x = previous->omega_hat.x + vec3_temp.x * dt;
    current->omega_hat.y = previous->omega_hat.y + vec3_temp.y * dt;
    current->omega_hat.z = previous->omega_hat.z + vec3_temp.z * dt;

    previous->vel_hat = current->vel_hat;      // update
    previous->omega_hat = current->omega_hat;  // update

    vector3f_t vpred_error = (vector3f_t){
        current->vel_hat.x - current->vel.x,
        current->vel_hat.y - current->vel.y,
        current->vel_hat.z - current->vel.z
    };
    vector3f_t Wpred_error = (vector3f_t){
        current->omega_hat.x - current->omega.x,
        current->omega_hat.y - current->omega.y,
        current->omega_hat.z - current->omega.z
    };

    float exp_As_v_dt = expf(l1_params->As_v * dt);
    float exp_As_W_dt = expf(l1_params->As_W * dt);

    vector3f_t PhiInvmu_v = (vector3f_t){
        vpred_error.x / (exp_As_v_dt - 1.0f) * l1_params->As_v * exp_As_v_dt,
        vpred_error.y / (exp_As_v_dt - 1.0f) * l1_params->As_v * exp_As_v_dt,
        vpred_error.z / (exp_As_v_dt - 1.0f) * l1_params->As_v * exp_As_v_dt
    };
    vector3f_t PhiInvmu_W = (vector3f_t){
        Wpred_error.x / (exp_As_W_dt - 1.0f) * l1_params->As_W * exp_As_W_dt,
        Wpred_error.y / (exp_As_W_dt - 1.0f) * l1_params->As_W * exp_As_W_dt,
        Wpred_error.z / (exp_As_W_dt - 1.0f) * l1_params->As_W * exp_As_W_dt
    };

    matrix3f_t R;
    QUADLIB_CHECK(quaternion_to_rotation_matrix(&R, &current->quat));

    QUADLIB_CHECK(vector3_dot(&scal1_temp, &R.colx, &PhiInvmu_v));
    current->sigma_f_hat.x = -scal1_temp * quad_params->mass;  // unmatched disturbance estimate (fx)
    QUADLIB_CHECK(vector3_dot(&scal1_temp, &R.coly, &PhiInvmu_v));
    current->sigma_f_hat.y = -scal1_temp * quad_params->mass;  // unmatched disturbance estimate (fy)
    QUADLIB_CHECK(vector3_dot(&scal1_temp, &R.colz, &PhiInvmu_v));
    current->sigma_f_hat.z = scal1_temp * quad_params->mass;   // matched disturbance estimate (fz)

    current->sigma_M_hat = (vector3f_t){
        -quad_params->inertia.x * PhiInvmu_W.x,                // matched disturbance estimate (Mx)
        -quad_params->inertia.y * PhiInvmu_W.y,                // matched disturbance estimate (My)
        -quad_params->inertia.z * PhiInvmu_W.z                 // matched disturbance estimate (Mz)
    };

    previous->sigma_f_hat = current->sigma_f_hat;  // update
    previous->sigma_M_hat = current->sigma_M_hat;  // update

    float lpf1_coeff_f1 = expf(-l1_params->lpf1_cutoffreq_f * 0.0025);
    float lpf1_coeff_f2 = 1.0 - lpf1_coeff_f1;
    float lpf1_coeff_M1 = expf(-l1_params->lpf1_cutoffreq_M * 0.0025);
    float lpf1_coeff_M2 = 1.0 - lpf1_coeff_M1;
    float lpf2_coeff_M1 = expf(-l1_params->lpf2_cutoffreq_M * 0.0025);
    float lpf2_coeff_M2 = 1.0 - lpf2_coeff_M1;
    
    current->lpf1.u1 = lpf1_coeff_f1 * previous->lpf1.u1 + lpf1_coeff_f2 * current->sigma_f_hat.z;
    current->lpf1.u2 = lpf1_coeff_M1 * previous->lpf1.u2 + lpf1_coeff_M2 * current->sigma_M_hat.x;
    current->lpf1.u3 = lpf1_coeff_M1 * previous->lpf1.u3 + lpf1_coeff_M2 * current->sigma_M_hat.y;
    current->lpf1.u4 = lpf1_coeff_M1 * previous->lpf1.u4 + lpf1_coeff_M2 * current->sigma_M_hat.z;

    previous->lpf1 = current->lpf1;  // update

    current->lpf2.u1 = current->lpf1.u1;
    current->lpf2.u2 = lpf2_coeff_M1 * previous->lpf2.u2 + lpf2_coeff_M2 * current->lpf1.u2;
    current->lpf2.u3 = lpf2_coeff_M1 * previous->lpf2.u3 + lpf2_coeff_M2 * current->lpf1.u3;
    current->lpf2.u4 = lpf2_coeff_M1 * previous->lpf2.u4 + lpf2_coeff_M2 * current->lpf1.u4;

    previous->lpf2 = current->lpf2;  // update

    current->uad = (control_4f_t){
        -current->lpf2.u1,  // adaptive thrust control (to cancel out the low-pass filtered matched disturbance estimate for force)
        -current->lpf2.u2,  // adaptive moment control (to cancel out the low-pass filtered matched disturbance estimate for Mx)
        -current->lpf2.u3,  // adaptive moment control (to cancel out the low-pass filtered matched disturbance estimate for My)
        -current->lpf2.u4   // adaptive moment control (to cancel out the low-pass filtered matched disturbance estimate for Mz)
    };

    previous->uad = current->uad;      // update
    previous->ub = current->ub;        // update
    previous->quat = current->quat;    // update
    previous->vel = current->vel;      // update
    previous->omega = current->omega;  // update

    return QUADLIB_SUCCESS;
}