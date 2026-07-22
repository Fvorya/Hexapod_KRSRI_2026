#include "BodyKinematics.h"
#include <math.h>

// Mengisi pData dari matriks identitas
void BodyKinematics::identity(arm_matrix_instance_f32* mat) {
    if (mat->numRows != 4 || mat->numCols != 4) return; // Proteksi
    memset(mat->pData, 0, sizeof(float) * 16);
    mat->pData[0] = 1.0f;  // [0][0]
    mat->pData[5] = 1.0f;  // [1][1]
    mat->pData[10] = 1.0f; // [2][2]
    mat->pData[15] = 1.0f; // [3][3]
}

// Mengisi pData dari matriks translasi
void BodyKinematics::translation(float x, float y, float z, 
                                 arm_matrix_instance_f32* mat) {
    identity(mat);
    mat->pData[3] = x;   // [0][3]
    mat->pData[7] = y;   // [1][3]
    mat->pData[11] = z;  // [2][3]
}

// Mengisi pData dari matriks rotasi X
void BodyKinematics::rotationX(float angle, arm_matrix_instance_f32* mat) {
    identity(mat);
    float cosA = cos(angle);
    float sinA = sin(angle);
    mat->pData[5] = cosA;   // [1][1]
    mat->pData[6] = -sinA;  // [1][2]
    mat->pData[9] = sinA;   // [2][1]
    mat->pData[10] = cosA;  // [2][2]
}

// Mengisi pData dari matriks rotasi Y
void BodyKinematics::rotationY(float angle, arm_matrix_instance_f32* mat) {
    identity(mat);
    float cosA = cos(angle);
    float sinA = sin(angle);
    mat->pData[0] = cosA;   // [0][0]
    mat->pData[2] = sinA;   // [0][2]
    mat->pData[8] = -sinA;  // [2][0]
    mat->pData[10] = cosA;  // [2][2]
}

// Mengisi pData dari matriks rotasi Z
void BodyKinematics::rotationZ(float angle, arm_matrix_instance_f32* mat) {
    identity(mat);
    float cosA = cos(angle);
    float sinA = sin(angle);
    mat->pData[0] = cosA;   // [0][0]
    mat->pData[1] = -sinA;  // [0][1]
    mat->pData[4] = sinA;   // [1][0]
    mat->pData[5] = cosA;   // [1][1]
}

// Menerapkan matriks ke vektor (Matrix[4x4] * Vector[4x1] = Vector[4x1])
void BodyKinematics::apply(const arm_matrix_instance_f32* mat, 
                           const arm_matrix_instance_f32* vecIn,
                           arm_matrix_instance_f32* vecOut) 
{
    // Ini adalah fungsi SIMD-accelerated dari CMSIS-DSP
    // Menggantikan perkalian 4x4 manual kita
    arm_mat_mult_f32(mat, vecIn, vecOut);
}
