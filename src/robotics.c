#include "../include/mini_quadlib.h"

// =============================================================================
// UTILITY FUNCTIONS: ROBOTICS TOOLS
// =============================================================================

quadlib_result_t frame_world_to_body(vector3f_t* output_body_vector,
                                     const vector3f_t* world_vector,
                                     const quaternion4f_t* attitude)
{
    // add check & normalization here

    matrix3f_t R;
    QUADLIB_CHECK(quaternion_to_rotation_matrix(&R, attitude));

    // Perform the transformation: body_vector = R^T * world_vector
    output_body_vector->x = R.colx.x * world_vector->x + R.colx.y * world_vector->y + R.colx.z * world_vector->z;
    output_body_vector->y = R.coly.x * world_vector->x + R.coly.y * world_vector->y + R.coly.z * world_vector->z;
    output_body_vector->z = R.colz.x * world_vector->x + R.colz.y * world_vector->y + R.colz.z * world_vector->z;

    // add normalization here
    
    return QUADLIB_SUCCESS;
}

quadlib_result_t frame_body_to_world(vector3f_t* output_world_vector,
                                     const vector3f_t* body_vector,
                                     const quaternion4f_t* attitude)
{
    // add check & normalization here

    matrix3f_t R;
    QUADLIB_CHECK(quaternion_to_rotation_matrix(&R, attitude));

    // Perform the transformation: world_vector = R * body_vector
    output_world_vector->x = R.colx.x * body_vector->x + R.coly.x * body_vector->y + R.colz.x * body_vector->z;
    output_world_vector->y = R.colx.y * body_vector->x + R.coly.y * body_vector->y + R.colz.y * body_vector->z;
    output_world_vector->z = R.colx.z * body_vector->x + R.coly.z * body_vector->y + R.colz.z * body_vector->z;

    // add normalization here

    return QUADLIB_SUCCESS;
}

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
                                           const char* from_frame)
{
    typedef struct {
        bool has_value;  // true if axis is assigned a value by input, false otherwise
        float value;
        const vector3f_t* orientation;
    } axis_t;

    axis_t* input_axes[3];
    axis_t* output_axes[3];
    
    // vectors for standard basis
    const vector3f_t north_vec = {0.0f, 1.0f, 0.0f};
    const vector3f_t east_vec  = {1.0f, 0.0f, 0.0f};
    const vector3f_t west_vec  = {-1.0f, 0.0f, 0.0f};
    const vector3f_t south_vec = {0.0f, -1.0f, 0.0f};
    const vector3f_t up_vec    = {0.0f, 0.0f, 1.0f};
    const vector3f_t down_vec  = {0.0f, 0.0f, -1.0f};

    // axis of the global model
    axis_t north_axis = {false, 0.0f, &north_vec};
    axis_t east_axis  = {false, 0.0f, &east_vec};
    axis_t west_axis  = {false, 0.0f, &west_vec};
    axis_t south_axis = {false, 0.0f, &south_vec};
    axis_t up_axis    = {false, 0.0f, &up_vec};
    axis_t down_axis  = {false, 0.0f, &down_vec};

    // Check validity of input frame and map to axes
    for (int i=0; i<3; ++i) {
        switch (from_frame[i]) {
            case 'E':
            case 'e':
                if (east_axis.has_value || west_axis.has_value) 
                {
                    return QUADLIB_ERROR_INVALID_INPUT_STRING;  // Duplicate axis
                }
                east_axis.value = ((float) input_coordinate_vector->x) * ((i==0) ? 1.0 : 0.0)
                                + ((float) input_coordinate_vector->y) * ((i==1) ? 1.0 : 0.0)
                                + ((float) input_coordinate_vector->z) * ((i==2) ? 1.0 : 0.0);
                east_axis.has_value = true;
                west_axis.value = ((float) input_coordinate_vector->x) * ((i==0) ? -1.0 : 0.0)
                                + ((float) input_coordinate_vector->y) * ((i==1) ? -1.0 : 0.0)
                                + ((float) input_coordinate_vector->z) * ((i==2) ? -1.0 : 0.0);
                west_axis.has_value = true;
                input_axes[i] = &east_axis;
                break;
            case 'N':
            case 'n':
                if (north_axis.has_value || south_axis.has_value) 
                {
                    return QUADLIB_ERROR_INVALID_INPUT_STRING;  // Duplicate axis
                }
                north_axis.value = ((float) input_coordinate_vector->x) * ((i==0) ? 1.0 : 0.0)
                                 + ((float) input_coordinate_vector->y) * ((i==1) ? 1.0 : 0.0)
                                 + ((float) input_coordinate_vector->z) * ((i==2) ? 1.0 : 0.0);
                north_axis.has_value = true;
                south_axis.value = ((float) input_coordinate_vector->x) * ((i==0) ? -1.0 : 0.0)
                                 + ((float) input_coordinate_vector->y) * ((i==1) ? -1.0 : 0.0)
                                 + ((float) input_coordinate_vector->z) * ((i==2) ? -1.0 : 0.0);
                south_axis.has_value = true;
                input_axes[i] = &north_axis;
                break;
            case 'W':
            case 'w':
                if (west_axis.has_value || east_axis.has_value) 
                {
                    return QUADLIB_ERROR_INVALID_INPUT_STRING;  // Duplicate axis
                }
                west_axis.value = ((float) input_coordinate_vector->x) * ((i==0) ? 1.0 : 0.0)
                                + ((float) input_coordinate_vector->y) * ((i==1) ? 1.0 : 0.0)
                                + ((float) input_coordinate_vector->z) * ((i==2) ? 1.0 : 0.0);
                west_axis.has_value = true;
                east_axis.value = ((float) input_coordinate_vector->x) * ((i==0) ? -1.0 : 0.0)
                                + ((float) input_coordinate_vector->y) * ((i==1) ? -1.0 : 0.0)
                                + ((float) input_coordinate_vector->z) * ((i==2) ? -1.0 : 0.0);
                east_axis.has_value = true;
                input_axes[i] = &west_axis;
                break;
            case 'S':
            case 's':
                if (south_axis.has_value || north_axis.has_value) 
                {
                    return QUADLIB_ERROR_INVALID_INPUT_STRING;  // Duplicate axis
                }
                south_axis.value = ((float) input_coordinate_vector->x) * ((i==0) ? 1.0 : 0.0)
                                 + ((float) input_coordinate_vector->y) * ((i==1) ? 1.0 : 0.0)
                                 + ((float) input_coordinate_vector->z) * ((i==2) ? 1.0 : 0.0);
                south_axis.has_value = true;
                north_axis.value = ((float) input_coordinate_vector->x) * ((i==0) ? -1.0 : 0.0)
                                 + ((float) input_coordinate_vector->y) * ((i==1) ? -1.0 : 0.0)
                                 + ((float) input_coordinate_vector->z) * ((i==2) ? -1.0 : 0.0);
                north_axis.has_value = true;
                input_axes[i] = &south_axis;
                break;
            case 'U':
            case 'u':
                if (up_axis.has_value || down_axis.has_value) 
                {
                    return QUADLIB_ERROR_INVALID_INPUT_STRING;  // Duplicate axis
                }
                up_axis.value = ((float) input_coordinate_vector->x) * ((i==0) ? 1.0 : 0.0)
                              + ((float) input_coordinate_vector->y) * ((i==1) ? 1.0 : 0.0)
                              + ((float) input_coordinate_vector->z) * ((i==2) ? 1.0 : 0.0);
                up_axis.has_value = true;
                down_axis.value = ((float) input_coordinate_vector->x) * ((i==0) ? -1.0 : 0.0)
                                + ((float) input_coordinate_vector->y) * ((i==1) ? -1.0 : 0.0)
                                + ((float) input_coordinate_vector->z) * ((i==2) ? -1.0 : 0.0);
                down_axis.has_value = true;
                input_axes[i] = &up_axis;
                break;
            case 'D':
            case 'd':
                if (down_axis.has_value || up_axis.has_value) 
                {
                    return QUADLIB_ERROR_INVALID_INPUT_STRING;  // Duplicate axis
                }
                down_axis.value = ((float) input_coordinate_vector->x) * ((i==0) ? 1.0 : 0.0)
                                + ((float) input_coordinate_vector->y) * ((i==1) ? 1.0 : 0.0)
                                + ((float) input_coordinate_vector->z) * ((i==2) ? 1.0 : 0.0);
                down_axis.has_value = true;
                up_axis.value = ((float) input_coordinate_vector->x) * ((i==0) ? -1.0 : 0.0)
                              + ((float) input_coordinate_vector->y) * ((i==1) ? -1.0 : 0.0)
                              + ((float) input_coordinate_vector->z) * ((i==2) ? -1.0 : 0.0);
                up_axis.has_value = true;
                input_axes[i] = &down_axis;
                break;
            default:
                return QUADLIB_ERROR_INVALID_INPUT_STRING;
        }
    }

    // Check validity of output frame and get values from mapped global model
    for (int i=0; i<3; ++i) {
        switch (to_frame[i]) {
            case 'E':
            case 'e':
                output_axes[i] = &east_axis;
                break;
            case 'N':
            case 'n':
                output_axes[i] = &north_axis;
                break;
            case 'W':
            case 'w':
                output_axes[i] = &west_axis;
                break;
            case 'S':
            case 's':
                output_axes[i] = &south_axis;
                break;
            case 'U':
            case 'u':
                output_axes[i] = &up_axis;
                break;
            case 'D':
            case 'd':
                output_axes[i] = &down_axis;
                break;
            default:
                return QUADLIB_ERROR_INVALID_INPUT_STRING;
        }
    }

    vector3f_t input_x_cross_y;
    vector3f_t output_x_cross_y;
    bool is_input_right_handed = true;
    bool is_output_right_handed = true;

    QUADLIB_CHECK(vector3_cross(&input_x_cross_y, input_axes[0]->orientation, input_axes[1]->orientation));
    QUADLIB_CHECK(vector3_cross(&output_x_cross_y, output_axes[0]->orientation, output_axes[1]->orientation));

    QUADLIB_CHECK(vector3_is_equalf(&is_input_right_handed, &input_x_cross_y, input_axes[2]->orientation));
    QUADLIB_CHECK(vector3_is_equalf(&is_output_right_handed, &output_x_cross_y, output_axes[2]->orientation));

    if (!is_input_right_handed || !is_output_right_handed)
    {
        return QUADLIB_ERROR_SAFETY_VIOLATION;  // if not both frames are right-handed
    }

    output_coordinate_vector->x = output_axes[0]->value;
    output_coordinate_vector->y = output_axes[1]->value;
    output_coordinate_vector->z = output_axes[2]->value;

    return QUADLIB_SUCCESS;
}

quadlib_result_t enu_to_ned(vector3f_t* output_coordinate_vector,
                            const vector3f_t* input_coordinate_vector)
{
    output_coordinate_vector->x = input_coordinate_vector->y;
    output_coordinate_vector->y = input_coordinate_vector->x;
    output_coordinate_vector->z = -input_coordinate_vector->z;

    return QUADLIB_SUCCESS;
}

quadlib_result_t ned_to_enu(vector3f_t* output_coordinate_vector,
                            const vector3f_t* input_coordinate_vector)
{
    output_coordinate_vector->x = input_coordinate_vector->y;
    output_coordinate_vector->y = input_coordinate_vector->x;
    output_coordinate_vector->z = -input_coordinate_vector->z;

    return QUADLIB_SUCCESS;
}

quadlib_result_t quaternion_normalize(quaternion4f_t* output_q, const quaternion4f_t* q)
{
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);

    if (norm < QUADLIB_EPSILON) 
    {
        return QUADLIB_ERROR_INVALID_INPUT_NUMBER;
    }

    output_q->w = q->w / norm;
    output_q->x = q->x / norm;
    output_q->y = q->y / norm;
    output_q->z = q->z / norm;

    return QUADLIB_SUCCESS;
}

quadlib_result_t rotation_matrix_normalize(matrix3f_t* output_R, const matrix3f_t* R)
{
    /*
    Gram-Schmidt process for orthonormalizing the columns of a rotation matrix

    u1 = col1 / np.linalg.norm(col1)
    
    u2 = col2 - np.dot(col2, u1) * u1
    u2 = u2 / np.linalg.norm(u2)
    
    u3 = col3 - np.dot(col3, u1) * u1 - np.dot(col3, u2) * u2
    u3 = u3 / np.linalg.norm(u3)
    */ 
    float dot_temp;

    QUADLIB_CHECK(vector3_normalize(&output_R->colx, &R->colx));

    QUADLIB_CHECK(vector3_dot(&dot_temp, &R->coly, &output_R->colx));
    QUADLIB_CHECK(vector3_sub(&output_R->coly, &R->coly, &(vector3f_t){output_R->colx.x * dot_temp, output_R->colx.y * dot_temp, output_R->colx.z * dot_temp}));
    QUADLIB_CHECK(vector3_normalize(&output_R->coly, &output_R->coly));

    QUADLIB_CHECK(vector3_dot(&dot_temp, &R->colz, &output_R->colx));
    QUADLIB_CHECK(vector3_sub(&output_R->colz, &R->colz, &(vector3f_t){output_R->colx.x * dot_temp, output_R->colx.y * dot_temp, output_R->colx.z * dot_temp}));
    QUADLIB_CHECK(vector3_dot(&dot_temp, &output_R->colz, &output_R->coly));
    QUADLIB_CHECK(vector3_sub(&output_R->colz, &output_R->colz, &(vector3f_t){output_R->coly.x * dot_temp, output_R->coly.y * dot_temp, output_R->coly.z * dot_temp}));
    QUADLIB_CHECK(vector3_normalize(&output_R->colz, &output_R->colz));

    vector3f_t x_cross_y;
    bool is_right_handed = true;

    QUADLIB_CHECK(vector3_cross(&x_cross_y, &output_R->colx, &output_R->coly));
    QUADLIB_CHECK(vector3_is_equalf(&is_right_handed, &x_cross_y, &output_R->colz));

    if (!is_right_handed)
    {
        return QUADLIB_ERROR_SAFETY_VIOLATION;  // if not right-handed
    }

    return QUADLIB_SUCCESS;
}

quadlib_result_t euler_angles_normalize(vector3f_t* output_euler_angles, const vector3f_t* euler_angles)
{
    // Normalize Euler angles to standard ranges (-pi, pi]
    output_euler_angles->x = fmodf(euler_angles->x + QUADLIB_PI, 2.0f * QUADLIB_PI) - QUADLIB_PI;
    output_euler_angles->y = fmodf(euler_angles->y + QUADLIB_PI, 2.0f * QUADLIB_PI) - QUADLIB_PI;
    output_euler_angles->z = fmodf(euler_angles->z + QUADLIB_PI, 2.0f * QUADLIB_PI) - QUADLIB_PI;

    // Switch from -pi to pi
    if (output_euler_angles->x <= -QUADLIB_PI) 
    {
        output_euler_angles->x += 2.0f * QUADLIB_PI;
    }
    if (output_euler_angles->y <= -QUADLIB_PI) 
    {
        output_euler_angles->y += 2.0f * QUADLIB_PI;
    }
    if (output_euler_angles->z <= -QUADLIB_PI) 
    {
        output_euler_angles->z += 2.0f * QUADLIB_PI;
    }

    return QUADLIB_SUCCESS;
}

quadlib_result_t quaternion_multiply(quaternion4f_t* output_q, const quaternion4f_t* q1, const quaternion4f_t* q2)
{
    // Hamilton product of two quaternions, NOT commutative! 
    quaternion4f_t q1_normalized;
    quaternion4f_t q2_normalized;
    QUADLIB_CHECK(quaternion_normalize(&q1_normalized, q1));
    QUADLIB_CHECK(quaternion_normalize(&q2_normalized, q2));

    output_q->w = q1_normalized.w * q2_normalized.w - q1_normalized.x * q2_normalized.x - q1_normalized.y * q2_normalized.y - q1_normalized.z * q2_normalized.z;
    output_q->x = q1_normalized.w * q2_normalized.x + q1_normalized.x * q2_normalized.w + q1_normalized.y * q2_normalized.z - q1_normalized.z * q2_normalized.y;
    output_q->y = q1_normalized.w * q2_normalized.y - q1_normalized.x * q2_normalized.z + q1_normalized.y * q2_normalized.w + q1_normalized.z * q2_normalized.x;
    output_q->z = q1_normalized.w * q2_normalized.z + q1_normalized.x * q2_normalized.y - q1_normalized.y * q2_normalized.x + q1_normalized.z * q2_normalized.w;

    return QUADLIB_SUCCESS;
}

quadlib_result_t quaternion_conjugate(quaternion4f_t* output_q, const quaternion4f_t* q)
{
    output_q->w = q->w;
    output_q->x = -q->x;
    output_q->y = -q->y;
    output_q->z = -q->z;

    QUADLIB_CHECK(quaternion_normalize(output_q, output_q));

    return QUADLIB_SUCCESS;
}

quadlib_result_t rotation_matrix_to_quaternion(quaternion4f_t* output_q, const matrix3f_t* R)
{
    matrix3f_t R_normalized;
    QUADLIB_CHECK(rotation_matrix_normalize(&R_normalized, R));

    float trace = R_normalized.colx.x + R_normalized.coly.y + R_normalized.colz.z;
    if (trace > 0.0f) 
    {
        float s = 0.5f / sqrtf(trace + 1.0f);
        output_q->w = 0.25f / s;
        output_q->x = (R_normalized.coly.z - R_normalized.colz.y) * s;
        output_q->y = (R_normalized.colz.x - R_normalized.colx.z) * s;
        output_q->z = (R_normalized.colx.y - R_normalized.coly.x) * s;
    } 
    else if (R_normalized.colx.x > R_normalized.coly.y && R_normalized.colx.x > R_normalized.colz.z) 
    {
        float s = 2.0f * sqrtf(1.0f + R_normalized.colx.x - R_normalized.coly.y - R_normalized.colz.z);
        output_q->w = (R_normalized.coly.z - R_normalized.colz.y) / s;
        output_q->x = 0.25f * s;
        output_q->y = (R_normalized.coly.x + R_normalized.colx.y) / s;
        output_q->z = (R_normalized.colz.x + R_normalized.colx.z) / s;
    } 
    else if (R_normalized.coly.y > R_normalized.colz.z) 
    {
        float s = 2.0f * sqrtf(1.0f + R_normalized.coly.y - R_normalized.colx.x - R_normalized.colz.z);
        output_q->w = (R_normalized.colz.x - R_normalized.colx.z) / s;
        output_q->x = (R_normalized.coly.x + R_normalized.colx.y) / s;
        output_q->y = 0.25f * s;
        output_q->z = (R_normalized.colz.y + R_normalized.coly.z) / s;
    } 
    else 
    {
        float s = 2.0f * sqrtf(1.0f + R_normalized.colz.z - R_normalized.colx.x - R_normalized.coly.y);
        output_q->w = (R_normalized.colx.y - R_normalized.coly.x) / s;
        output_q->x = (R_normalized.colz.x + R_normalized.colx.z) / s;
        output_q->y = (R_normalized.colz.y + R_normalized.coly.z) / s;
        output_q->z = 0.25f * s;
    }

    return QUADLIB_SUCCESS;
}

quadlib_result_t quaternion_to_rotation_matrix(matrix3f_t* output_R, const quaternion4f_t* q)
{
    quaternion4f_t q_normalized;
    QUADLIB_CHECK(quaternion_normalize(&q_normalized, q));

    output_R->colx.x = 1 - 2 * (q_normalized.y * q_normalized.y + q_normalized.z * q_normalized.z);
    output_R->coly.x = 2 * (q_normalized.x * q_normalized.y - q_normalized.z * q_normalized.w);
    output_R->colz.x = 2 * (q_normalized.x * q_normalized.z + q_normalized.y * q_normalized.w);

    output_R->colx.y = 2 * (q_normalized.x * q_normalized.y + q_normalized.z * q_normalized.w);
    output_R->coly.y = 1 - 2 * (q_normalized.x * q_normalized.x + q_normalized.z * q_normalized.z);
    output_R->colz.y = 2 * (q_normalized.y * q_normalized.z - q_normalized.x * q_normalized.w);

    output_R->colx.z = 2 * (q_normalized.x * q_normalized.z - q_normalized.y * q_normalized.w);
    output_R->coly.z = 2 * (q_normalized.y * q_normalized.z + q_normalized.x * q_normalized.w);
    output_R->colz.z = 1 - 2 * (q_normalized.x * q_normalized.x + q_normalized.y * q_normalized.y);

    return QUADLIB_SUCCESS;
}

quadlib_result_t euler_angles_to_rotation_matrix(matrix3f_t* output_R, const vector3f_t* euler_angles)
{
    float cr = cosf(euler_angles->x);  // roll
    float sr = sinf(euler_angles->x);
    float cp = cosf(euler_angles->y);  // pitch
    float sp = sinf(euler_angles->y);
    float cy = cosf(euler_angles->z);  // yaw
    float sy = sinf(euler_angles->z);

    output_R->colx.x = cp * cy;
    output_R->coly.x = sr * sp * cy - cr * sy;
    output_R->colz.x = cr * sp * cy + sr * sy;

    output_R->colx.y = cp * sy;
    output_R->coly.y = sr * sp * sy + cr * cy;
    output_R->colz.y = cr * sp * sy - sr * cy;

    output_R->colx.z = -sp;
    output_R->coly.z = sr * cp;
    output_R->colz.z = cr * cp;

    return QUADLIB_SUCCESS;
}

quadlib_result_t rotation_matrix_to_euler_angles(vector3f_t* output_euler_angles, const matrix3f_t* R)
{
    matrix3f_t R_normalized;
    QUADLIB_CHECK(rotation_matrix_normalize(&R_normalized, R));

    if (R_normalized.colx.z > -1.0f && R_normalized.colx.z < 1.0f)
    {
        output_euler_angles->y = -asinf(R_normalized.colx.z);  // pitch
        output_euler_angles->x = atan2f(R_normalized.coly.z, R_normalized.colz.z);  // roll
        output_euler_angles->z = atan2f(R_normalized.colx.y, R_normalized.colx.x);  // yaw
    }
    else if (R_normalized.colx.z > -1.0f - QUADLIB_EPSILON && R_normalized.colx.z < -1.0f + QUADLIB_EPSILON)
    {
        // R->colx.z == -1
        output_euler_angles->y = QUADLIB_PI_2;
        output_euler_angles->x = 0.0f;
        output_euler_angles->z = atan2f(R_normalized.coly.x, R_normalized.colz.x);
    }
    else if (R_normalized.colx.z > 1.0f - QUADLIB_EPSILON && R_normalized.colx.z < 1.0f + QUADLIB_EPSILON)
    {
        // R->colx.z == 1
        output_euler_angles->y = -QUADLIB_PI_2;
        output_euler_angles->x = 0.0f;
        output_euler_angles->z = atan2f(-R_normalized.coly.x, -R_normalized.colz.x);
    }
    
    return QUADLIB_SUCCESS;
}

quadlib_result_t rotation_matrix_transpose(matrix3f_t* output_RT, const matrix3f_t* R)
{
    matrix3f_t R_normalized;
    QUADLIB_CHECK(rotation_matrix_normalize(&R_normalized, R));

    output_RT->colx.x = R_normalized.colx.x;
    output_RT->colx.y = R_normalized.coly.x;
    output_RT->colx.z = R_normalized.colz.x;
    output_RT->coly.x = R_normalized.colx.y;
    output_RT->coly.y = R_normalized.coly.y;
    output_RT->coly.z = R_normalized.colz.y;
    output_RT->colz.x = R_normalized.colx.z;
    output_RT->colz.y = R_normalized.coly.z;
    output_RT->colz.z = R_normalized.colz.z;

    return QUADLIB_SUCCESS;
}