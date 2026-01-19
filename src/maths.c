#include "../include/mini_quadlib.h"

// =============================================================================
// UTILITY FUNCTIONS: NORMAL MATHS
// =============================================================================

quadlib_result_t vector3_is_equalf(bool* is_equal, vector3f_t* a, const vector3f_t* b)
{
    float diff_x = fabsf(a->x - b->x);
    float diff_y = fabsf(a->y - b->y);
    float diff_z = fabsf(a->z - b->z);

    *is_equal = (diff_x < QUADLIB_EPSILON) && (diff_y < QUADLIB_EPSILON) && (diff_z < QUADLIB_EPSILON);

    return QUADLIB_SUCCESS;
}

quadlib_result_t vector3_sub(vector3f_t* output_v, const vector3f_t* a, const vector3f_t* b)
{
    output_v->x = a->x - b->x;
    output_v->y = a->y - b->y;
    output_v->z = a->z - b->z;

    return QUADLIB_SUCCESS;
}

quadlib_result_t vector3_add(vector3f_t* output_v, const vector3f_t* a, const vector3f_t* b)
{
    output_v->x = a->x + b->x;
    output_v->y = a->y + b->y;
    output_v->z = a->z + b->z;

    return QUADLIB_SUCCESS;
}

quadlib_result_t vector3_get_norm(float* output_norm, const vector3f_t* v)
{
    *output_norm = sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);

    return QUADLIB_SUCCESS;
}

quadlib_result_t vector3_normalize(vector3f_t* output_v, const vector3f_t* v)
{
    float norm = sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);

    if (norm < QUADLIB_EPSILON) {
        return QUADLIB_ERROR_INVALID_INPUT_NUMBER; // Avoid division by zero
    }

    output_v->x = v->x / norm;
    output_v->y = v->y / norm;
    output_v->z = v->z / norm;

    return QUADLIB_SUCCESS;
}

quadlib_result_t vector3_dot(float* output_dot, const vector3f_t* a, const vector3f_t* b)
{
    *output_dot = a->x * b->x + a->y * b->y + a->z * b->z;

    return QUADLIB_SUCCESS;
}

quadlib_result_t vector3_dotcr(matrix3f_t* output_dot, const vector3f_t* a, const vector3f_t* b)
{
    output_dot->colx.x = a->x * b->x;
    output_dot->coly.x = a->x * b->y;
    output_dot->colz.x = a->x * b->z;

    output_dot->colx.y = a->y * b->x;
    output_dot->coly.y = a->y * b->y;
    output_dot->colz.y = a->y * b->z;

    output_dot->colx.z = a->z * b->x;
    output_dot->coly.z = a->z * b->y;
    output_dot->colz.z = a->z * b->z;

    return QUADLIB_SUCCESS;
}

quadlib_result_t vector3_dotm(vector3f_t* output_dot, const vector3f_t* v, const matrix3f_t* m)
{
    output_dot->x = v->x * m->colx.x + v->y * m->colx.y + v->z * m->colx.z;
    output_dot->y = v->x * m->coly.x + v->y * m->coly.y + v->z * m->coly.z;
    output_dot->z = v->x * m->colz.x + v->y * m->colz.y + v->z * m->colz.z;

    return QUADLIB_SUCCESS;
}

quadlib_result_t vector3_cross(vector3f_t* output_cross, const vector3f_t* a, const vector3f_t* b)
{
    output_cross->x = a->y * b->z - a->z * b->y;
    output_cross->y = a->z * b->x - a->x * b->z;
    output_cross->z = a->x * b->y - a->y * b->x;

    return QUADLIB_SUCCESS;
}

quadlib_result_t vector3_hat(matrix3f_t* output_matrix, const vector3f_t* v)
{
    output_matrix->colx.x = 0.0f;
    output_matrix->colx.y = v->z;
    output_matrix->colx.z = -v->y;

    output_matrix->coly.x = -v->z;
    output_matrix->coly.y = 0.0f;
    output_matrix->coly.z = v->x;

    output_matrix->colz.x = v->y;
    output_matrix->colz.y = -v->x;
    output_matrix->colz.z = 0.0f;

    return QUADLIB_SUCCESS;
}

quadlib_result_t vector3_wedge(matrix3f_t* output_matrix, const vector3f_t* v)
{
    return vector3_hat(output_matrix, v);
}

quadlib_result_t matrix3_is_equalf(bool* is_equal, matrix3f_t* a, const matrix3f_t* b)
{
    bool colx_equal, coly_equal, colz_equal;
    QUADLIB_CHECK(vector3_is_equalf(&colx_equal, &a->colx, &b->colx));
    QUADLIB_CHECK(vector3_is_equalf(&coly_equal, &a->coly, &b->coly));
    QUADLIB_CHECK(vector3_is_equalf(&colz_equal, &a->colz, &b->colz));

    *is_equal = colx_equal && coly_equal && colz_equal;

    return QUADLIB_SUCCESS;
}

quadlib_result_t matrix3_sub(matrix3f_t* output_dot, const matrix3f_t* a, const matrix3f_t* b)
{
    output_dot->colx.x = a->colx.x - b->colx.x;
    output_dot->colx.y = a->colx.y - b->colx.y;
    output_dot->colx.z = a->colx.z - b->colx.z;

    output_dot->coly.x = a->coly.x - b->coly.x;
    output_dot->coly.y = a->coly.y - b->coly.y;
    output_dot->coly.z = a->coly.z - b->coly.z;

    output_dot->colz.x = a->colz.x - b->colz.x;
    output_dot->colz.y = a->colz.y - b->colz.y;
    output_dot->colz.z = a->colz.z - b->colz.z;

    return QUADLIB_SUCCESS;
}

quadlib_result_t matrix3_add(matrix3f_t* output_dot, const matrix3f_t* a, const matrix3f_t* b)
{
    output_dot->colx.x = a->colx.x + b->colx.x;
    output_dot->colx.y = a->colx.y + b->colx.y;
    output_dot->colx.z = a->colx.z + b->colx.z;

    output_dot->coly.x = a->coly.x + b->coly.x;
    output_dot->coly.y = a->coly.y + b->coly.y;
    output_dot->coly.z = a->coly.z + b->coly.z;

    output_dot->colz.x = a->colz.x + b->colz.x;
    output_dot->colz.y = a->colz.y + b->colz.y;
    output_dot->colz.z = a->colz.z + b->colz.z;

    return QUADLIB_SUCCESS;
}

quadlib_result_t matrix3_dot(matrix3f_t* output_dot, const matrix3f_t* a, const matrix3f_t* b)
{
    output_dot->colx.x = a->colx.x * b->colx.x + a->coly.x * b->colx.y + a->colz.x * b->colx.z;
    output_dot->colx.y = a->colx.y * b->colx.x + a->coly.y * b->colx.y + a->colz.y * b->colx.z;
    output_dot->colx.z = a->colx.z * b->colx.x + a->coly.z * b->colx.y + a->colz.z * b->colx.z;

    output_dot->coly.x = a->colx.x * b->coly.x + a->coly.x * b->coly.y + a->colz.x * b->coly.z;
    output_dot->coly.y = a->colx.y * b->coly.x + a->coly.y * b->coly.y + a->colz.y * b->coly.z;
    output_dot->coly.z = a->colx.z * b->coly.x + a->coly.z * b->coly.y + a->colz.z * b->coly.z;

    output_dot->colz.x = a->colx.x * b->colz.x + a->coly.x * b->colz.y + a->colz.x * b->colz.z;
    output_dot->colz.y = a->colx.y * b->colz.x + a->coly.y * b->colz.y + a->colz.y * b->colz.z;
    output_dot->colz.z = a->colx.z * b->colz.x + a->coly.z * b->colz.y + a->colz.z * b->colz.z;

    return QUADLIB_SUCCESS;
}

quadlib_result_t matrix3_dotv(vector3f_t* output_dot, const matrix3f_t* m, const vector3f_t* v)
{
    output_dot->x = m->colx.x * v->x + m->coly.x * v->y + m->colz.x * v->z;
    output_dot->y = m->colx.y * v->x + m->coly.y * v->y + m->colz.y * v->z;
    output_dot->z = m->colx.z * v->x + m->coly.z * v->y + m->colz.z * v->z;

    return QUADLIB_SUCCESS;
}

quadlib_result_t matrix3_vee(vector3f_t* output_v, const matrix3f_t* M)
{
    output_v->x = (M->coly.z - M->colz.y) / 2.0f;
    output_v->y = (M->colz.x - M->colx.z) / 2.0f;
    output_v->z = (M->colx.y - M->coly.x) / 2.0f;

    return QUADLIB_SUCCESS;
}