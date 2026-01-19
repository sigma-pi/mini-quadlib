#include "../include/mini_quadlib.h"

// =============================================================================
// TEST FUNCTIONS
// =============================================================================

int main()
{
    // Test maths functions
    {
        // vector3f_t body = {1.0f, 2.0f, 3.0f};
        // vector3f_t world = {0.0f, 0.0f, 1.0f};

        // QUADLIB_CHECK(vector3_normalize(&body, &body));
        // printf("Normalized body vector: [%f, %f, %f]\n", body.x, body.y, body.z);

        // vector3f_t zero = {0.0f, 0.0f, 0.0f};
        // // QUADLIB_CHECK(vector3_normalize(&zero, &zero));
        // // printf("Normalized zero vector: [%f, %f, %f]\n", zero.x, zero.y, zero.z);

        // vector3f_t halfeps = {QUADLIB_EPSILON/2.0f, QUADLIB_EPSILON/2.0f, QUADLIB_EPSILON/2.0f};
        // // QUADLIB_CHECK(vector3_normalize(&halfeps, &halfeps));
        // // printf("Normalized zero vector: [%f, %f, %f]\n", halfeps.x, halfeps.y, halfeps.z);

        // vector3f_t added = {};
        // QUADLIB_CHECK(vector3_add(&added, &body, &halfeps));
        // printf("Added vector: [%f, %f, %f]\n", added.x, added.y, added.z);

        // vector3f_t subbed = {};
        // QUADLIB_CHECK(vector3_sub(&subbed, &body, &world));
        // printf("Subtracted vector (body - world): [%f, %f, %f]\n", subbed.x, subbed.y, subbed.z);

        // bool is_equal;
        // QUADLIB_CHECK(vector3_is_equalf(&is_equal, &body, &added));
        // printf("Are body and added vectors equal? %s\n", is_equal ? "Yes" : "No");

        // float deg = 180.0f;
        // float rad = QUADLIB_DEG_TO_RAD(deg);
        // printf("%f degrees is %f radians\n", deg, rad);
        // deg = QUADLIB_RAD_TO_DEG(rad);
        // printf("%f radians is %f degrees\n", rad, deg);

        // vector3f_t v1 = {1.0f, 2.0f, 3.0f};
        // vector3f_t v2 = {4.0f, 5.0f, 6.0f};

        // float dot_product;
        // QUADLIB_CHECK(vector3_dot(&dot_product, &v1, &v2));
        // printf("Dot product of v1 and v2: %f\n", dot_product);

        // vector3f_t cross_product;
        // QUADLIB_CHECK(vector3_cross(&cross_product, &v1, &v2));
        // printf("Cross product of v1 and v2: [%f, %f, %f]\n", cross_product.x, cross_product.y, cross_product.z);

        // vector3f_t dot_vec;
        // matrix3f_t hat_matrix;
        // QUADLIB_CHECK(vector3_hat(&hat_matrix, &v1));
        // QUADLIB_CHECK(matrix3_dotv(&dot_vec, &hat_matrix, &v2));
        // printf("Dot product of hat_matrix and v2: [%f, %f, %f]\n", dot_vec.x, dot_vec.y, dot_vec.z);
        // QUADLIB_CHECK(vector3_is_equalf(&is_equal, &dot_vec, &cross_product));
        // printf("Is dot_vec equal to cross_product? %s\n", is_equal ? "Yes" : "No");

        // QUADLIB_CHECK(matrix3_vee(&dot_vec, &hat_matrix));
        // printf("Vee of hat_matrix: [%f, %f, %f]\n", dot_vec.x, dot_vec.y, dot_vec.z);
        // QUADLIB_CHECK(vector3_is_equalf(&is_equal, &dot_vec, &v1));
        // printf("Is vee(hat_matrix) equal to v1? %s\n", is_equal ? "Yes" : "No");

        // matrix3f_t m1 = {
        //     .colx = {1.0f, 2.0f, 3.0f},
        //     .coly = {4.0f, 5.0f, 6.0f},
        //     .colz = {7.0f, 8.0f, 9.0f}
        // };
        // matrix3f_t m2 = {
        //     .colx = {9.0f, 8.0f, 7.0f},
        //     .coly = {6.0f, 5.0f, 4.0f},
        //     .colz = {3.0f, 2.0f, 1.0f}
        // };

        // matrix3f_t m_dot;
        // QUADLIB_CHECK(matrix3_dot(&m_dot, &m1, &m2));
        // printf("Dot product of m1 and m2:\n");
        // printf("| %f %f %f |\n", m_dot.colx.x, m_dot.coly.x, m_dot.colz.x);
        // printf("| %f %f %f |\n", m_dot.colx.y, m_dot.coly.y, m_dot.colz.y);
        // printf("| %f %f %f |\n", m_dot.colx.z, m_dot.coly.z, m_dot.colz.z);

        // QUADLIB_CHECK(matrix3_dotv(&dot_vec, &m1, &v1));
        // printf("Dot product of m1 and v1: [%f, %f, %f]\n", dot_vec.x, dot_vec.y, dot_vec.z);

        // QUADLIB_CHECK(vector3_dotm(&dot_vec, &v1, &m1));
        // printf("Dot product of v1 and m1: [%f, %f, %f]\n", dot_vec.x, dot_vec.y, dot_vec.z);

        // QUADLIB_CHECK(vector3_dotcr(&m_dot, &v1, &v2));
        // printf("Dot product of v1 (col) and v2 (row):\n");
        // printf("| %f %f %f |\n", m_dot.colx.x, m_dot.coly.x, m_dot.colz.x);
        // printf("| %f %f %f |\n", m_dot.colx.y, m_dot.coly.y, m_dot.colz.y);
        // printf("| %f %f %f |\n", m_dot.colx.z, m_dot.coly.z, m_dot.colz.z);
    }
    
    // Test robotics functions
    {
        vector3f_t body = {1.0f, 2.0f, 3.0f};

        vector3f_t angles_deg = {30, -45, 60}; // roll, pitch, yaw
        vector3f_t angles_rad = {
            QUADLIB_DEG_TO_RAD(angles_deg.x),
            QUADLIB_DEG_TO_RAD(angles_deg.y),
            QUADLIB_DEG_TO_RAD(angles_deg.z)
        };
        printf("Euler angles (deg): Roll: %f, Pitch: %f, Yaw: %f\n", angles_deg.x, angles_deg.y, angles_deg.z);
        printf("Euler angles (rad): Roll: %f, Pitch: %f, Yaw: %f\n", angles_rad.x, angles_rad.y, angles_rad.z);
        matrix3f_t R;
        quaternion4f_t q;

        QUADLIB_CHECK(euler_angles_to_rotation_matrix(&R, &angles_rad));
        printf("Rotation matrix from Euler angles (deg):\n");
        printf("| %f %f %f |\n", R.colx.x, R.coly.x, R.colz.x);
        printf("| %f %f %f |\n", R.colx.y, R.coly.y, R.colz.y);
        printf("| %f %f %f |\n", R.colx.z, R.coly.z, R.colz.z);

        QUADLIB_CHECK(rotation_matrix_to_quaternion(&q, &R));
        printf("Quaternion from rotation matrix: (w: %f, x: %f, y: %f, z: %f)\n", q.w, q.x, q.y, q.z);

        matrix3f_t R_from_q;
        QUADLIB_CHECK(quaternion_to_rotation_matrix(&R_from_q, &q));
        printf("Rotation matrix from quaternion:\n");
        printf("| %f %f %f |\n", R_from_q.colx.x, R_from_q.coly.x, R_from_q.colz.x);
        printf("| %f %f %f |\n", R_from_q.colx.y, R_from_q.coly.y, R_from_q.colz.y);
        printf("| %f %f %f |\n", R_from_q.colx.z, R_from_q.coly.z, R_from_q.colz.z);

        vector3f_t angles_rad_back;
        QUADLIB_CHECK(rotation_matrix_to_euler_angles(&angles_rad_back, &R));
        vector3f_t angles_deg_back = {
            QUADLIB_RAD_TO_DEG(angles_rad_back.x),
            QUADLIB_RAD_TO_DEG(angles_rad_back.y),
            QUADLIB_RAD_TO_DEG(angles_rad_back.z)
        };
        printf("Back-converted Euler angles (deg): Roll: %f, Pitch: %f, Yaw: %f\n", angles_deg_back.x, angles_deg_back.y, angles_deg_back.z);

        bool is_equal;
        QUADLIB_CHECK(matrix3_is_equalf(&is_equal, &R, &R_from_q));
        printf("Is back-converted matrix equal to original? %s\n", is_equal ? "Yes" : "No");

        vector3f_t world;
        vector3f_t body_back;
        vector3f_t dot_vec;

        QUADLIB_CHECK(frame_body_to_world(&world, &body, &q));
        printf("Body to world: [%f, %f, %f]\n", world.x, world.y, world.z);

        QUADLIB_CHECK(frame_world_to_body(&body_back, &world, &q));
        printf("World to body: [%f, %f, %f]\n", body_back.x, body_back.y, body_back.z);

        bool is_equal_vectors;
        QUADLIB_CHECK(vector3_is_equalf(&is_equal_vectors, &body, &body_back));
        printf("Is back-converted body vector equal to original? %s\n", is_equal_vectors ? "Yes" : "No");

        QUADLIB_CHECK(quaternion_to_rotation_matrix(&R_from_q, &q));
        QUADLIB_CHECK(matrix3_dotv(&dot_vec, &R_from_q, &body));
        printf("Dot product of rotation matrix and body vector: [%f, %f, %f]\n", dot_vec.x, dot_vec.y, dot_vec.z);
        QUADLIB_CHECK(vector3_is_equalf(&is_equal_vectors, &dot_vec, &world));
        printf("Is dot product equal to world vector? %s\n", is_equal_vectors ? "Yes" : "No");

        QUADLIB_CHECK(rotation_matrix_transpose(&R_from_q, &R_from_q));
        QUADLIB_CHECK(matrix3_dotv(&dot_vec, &R_from_q, &world));
        printf("Dot product of transposed rotation matrix and world vector: [%f, %f, %f]\n", dot_vec.x, dot_vec.y, dot_vec.z);
        QUADLIB_CHECK(vector3_is_equalf(&is_equal_vectors, &dot_vec, &body));
        printf("Is dot product equal to body vector? %s\n", is_equal_vectors ? "Yes" : "No");

        vector3f_t vec_ned = {1.0f, 2.0f, 3.0f};
        vector3f_t vec_enu;
        // QUADLIB_CHECK(ned_to_enu(&vec_enu, &vec_ned));
        // printf("NED to ENU: [%f, %f, %f]\n", vec_enu.x, vec_enu.y, vec_enu.z);
        // vector3f_t vec_ned_back;
        // QUADLIB_CHECK(enu_to_ned(&vec_ned_back, &vec_enu));
        // printf("ENU to NED: [%f, %f, %f]\n", vec_ned_back.x, vec_ned_back.y, vec_ned_back.z);
        // QUADLIB_CHECK(vector3_is_equalf(&is_equal_vectors, &vec_ned, &vec_ned_back));
        // printf("Is back-converted NED vector equal to original? %s\n", is_equal_vectors ? "Yes" : "No");

        // QUADLIB_CHECK(coordinate_transform_omni(&vec_enu, "enu", &vec_ned, "ned"));
        // printf("NED to ENU (omni): [%f, %f, %f]\n", vec_enu.x, vec_enu.y, vec_enu.z);
        // vector3f_t vec_ned_back;
        // QUADLIB_CHECK(coordinate_transform_omni(&vec_ned_back, "ned", &vec_enu, "enu"));
        // printf("ENU to NED (omni): [%f, %f, %f]\n", vec_ned_back.x, vec_ned_back.y, vec_ned_back.z);
        // QUADLIB_CHECK(vector3_is_equalf(&is_equal_vectors, &vec_ned, &vec_ned_back));
        // printf("Is back-converted NED vector equal to original? %s\n", is_equal_vectors ? "Yes" : "No");

        vector3f_t vec;
        QUADLIB_CHECK(coordinate_transform_omni(&vec, "NwU", &vec_ned, "neD"));
        printf("NED to NWU (omni): [%f, %f, %f]\n", vec.x, vec.y, vec.z);

        vector3f_t ang_deg = {270, -180, 530};
        vector3f_t ang_rad = {
            QUADLIB_DEG_TO_RAD(ang_deg.x),
            QUADLIB_DEG_TO_RAD(ang_deg.y),
            QUADLIB_DEG_TO_RAD(ang_deg.z)
        };
        vector3f_t ang_rad_normalized;
        printf("Euler angles (rad): Roll: %f, Pitch: %f, Yaw: %f\n", ang_rad.x, ang_rad.y, ang_rad.z);
        QUADLIB_CHECK(euler_angles_normalize(&ang_rad_normalized, &ang_rad));
        printf("Normalized Euler angles (rad): Roll: %f, Pitch: %f, Yaw: %f\n", ang_rad_normalized.x, ang_rad_normalized.y, ang_rad_normalized.z);

        quaternion4f_t dq = {0.8, 0.3, 0.2, 0.4};
        QUADLIB_CHECK(quaternion_normalize(&dq, &dq));
        printf("Normalized Quaternion: w: %f, x: %f, y: %f, z: %f\n", dq.w, dq.x, dq.y, dq.z);
        printf("Quaternion: w: %f, x: %f, y: %f, z: %f\n", q.w, q.x, q.y, q.z);

        quaternion4f_t qres;
        QUADLIB_CHECK(quaternion_multiply(&qres, &dq, &q));
        printf("Quaternion multiplication result: w: %f, x: %f, y: %f, z: %f\n", qres.w, qres.x, qres.y, qres.z);
    }

    return 0;
}