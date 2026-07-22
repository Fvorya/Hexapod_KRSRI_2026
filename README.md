# Ver 1.1 ☝🏽

Nambahin Body Kinematics dari program lama (LEGACY2026)
``` cpp
// ini di Hexapod.cpp

void Hexapod::solvePose() {
    // =================================================================
    // 1A) PERSIAPAN MATRIKS (Di luar loop agar tidak dihitung berulang)
    // =================================================================
    arm_matrix_instance_f32 matTrans, matRotX, matRotY, matRotZ;
    float dataTrans[16], dataRotX[16], dataRotY[16], dataRotZ[16];
    
    // Inisialisasi struktur matriks ARM (4 baris x 4 kolom)
    arm_mat_init_f32(&matTrans, 4, 4, dataTrans);
    arm_mat_init_f32(&matRotX, 4, 4, dataRotX);
    arm_mat_init_f32(&matRotY, 4, 4, dataRotY);
    arm_mat_init_f32(&matRotZ, 4, 4, dataRotZ);

    // Isi matriks dengan nilai NEGATIF (Kompensasi / Inverse Kinematics Bodi)
    BodyKinematics::translation(-_trans.x, -_trans.y, -_trans.z, &matTrans);
    BodyKinematics::rotationX(-_roll, &matRotX);
    BodyKinematics::rotationY(-_pitch, &matRotY);
    BodyKinematics::rotationZ(-_yaw, &matRotZ);

    for (int leg = 0; leg < 6; leg++) {
        Vec3 foot = _gait.legTargets[leg];

        // =================================================================
        // 1B) EKSEKUSI TRANSFORMASI SIMD (Pada masing-masing kaki)
        // =================================================================
        // Vektor harus berukuran 4x1 (Homogeneous Coordinate), diakhiri angka 1.0f
        float dataVecIn[4] = { foot.x, foot.y, foot.z, 1.0f }; 
        float dataVecOut[4] = { 0 };
        
        arm_matrix_instance_f32 vecIn, vecOut;
        arm_mat_init_f32(&vecIn, 4, 1, dataVecIn);
        arm_mat_init_f32(&vecOut, 4, 1, dataVecOut);

        // A. Terapkan Translasi
        BodyKinematics::apply(&matTrans, &vecIn, &vecOut);
        
        // B. Terapkan Rotasi X (Roll)
        memcpy(dataVecIn, dataVecOut, sizeof(dataVecIn)); // Pindah hasil ke input
        BodyKinematics::apply(&matRotX, &vecIn, &vecOut);
        
        // C. Terapkan Rotasi Y (Pitch)
        memcpy(dataVecIn, dataVecOut, sizeof(dataVecIn));
        BodyKinematics::apply(&matRotY, &vecIn, &vecOut);
        
        // D. Terapkan Rotasi Z (Yaw)
        memcpy(dataVecIn, dataVecOut, sizeof(dataVecIn));
        BodyKinematics::apply(&matRotZ, &vecIn, &vecOut);

        // Ekstrak kembali hasil akhir ke dalam bentuk Vec3 3D biasa
        Vec3 pb = { dataVecOut[0], dataVecOut[1], dataVecOut[2] };

        // =================================================================
        // 2) Relatif pangkal coxa (Tetap sama seperti sebelumnya)
        // =================================================================
        float vx = pb.x - BODY_LEG_ORIGINS[leg][0];
        float vy = pb.y - BODY_LEG_ORIGINS[leg][1];
        float vz = pb.z - BODY_LEG_ORIGINS[leg][2];

        // 3) Rotasi ke frame kaki (neutral menghadap +X)
        float a = -deg2rad(BODY_LEG_ANGLE[leg]);
        float lx = cosf(a) * vx - sinf(a) * vy;
        float ly = sinf(a) * vx + cosf(a) * vy;
        float lz = vz;

        // 4) Inverse Kinematics Kaki
        float coxa, femur, tibia;
        InverseKinematics::solve(lx, ly, lz, coxa, femur, tibia);

        // 5) Konversi ke pulse servo
        uint8_t c = leg * 3 + 0, f = leg * 3 + 1, t = leg * 3 + 2;
        _servos.setLegPulse(c, angleToPulse(c, coxa,          90.0f));
        _servos.setLegPulse(f, angleToPulse(f, femur,         90.0f));
        _servos.setLegPulse(t, angleToPulse(t, tibia - 90.0f, 90.0f));
    }
}
```
float vx, vy, vz <- ini buat ngasih tau jarak pangkal coxa ke target

float a  <- ini buat ngadepin sumbu x ke depan tiap masing-masing kaki
float lx <- ini buat ngasih tau kemana harusnya kaki saat ini melangkah di sumbu x jika diperintah (x,y)
float ly <- ini di sumbu y

file .ino-nya udah diganti buat ngetes body kinematics dari serial monitor
