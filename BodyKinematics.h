#ifndef BODYKINEMATICS_H
#define BODYKINEMATICS_H

#include <Arduino.h>
#include <arm_math.h> // PUSTAKA KUNCI UNTUK SIMD

class BodyKinematics {
public:
    // Mengisi 'mat' dengan matriks identitas
    static void identity(arm_matrix_instance_f32* mat);

    // Mengisi 'mat' dengan matriks translasi
    static void translation(float x, float y, float z, 
                            arm_matrix_instance_f32* mat);

    // Mengisi 'mat' dengan matriks rotasi
    static void rotationX(float angle, arm_matrix_instance_f32* mat);
    static void rotationY(float angle, arm_matrix_instance_f32* mat);
    static void rotationZ(float angle, arm_matrix_instance_f32* mat);

    // Mengalikan matriks 4x4 ke titik (x, y, z)
    // Ini adalah pengganti 'apply'
    // (vecIn harus [x, y, z, 1.0])
    static void apply(const arm_matrix_instance_f32* mat, 
                      const arm_matrix_instance_f32* vecIn,
                      arm_matrix_instance_f32* vecOut);
};

#endif
