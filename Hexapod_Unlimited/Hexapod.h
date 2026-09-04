#ifndef HEXAPOD_H
#define HEXAPOD_H

// Fasad: SATU-SATUNYA class yang dipakai di .ino & misi.
// Mengurus: gait -> body transform -> IK per kaki -> pulse servo (terkalibrasi).
#include "EEPROM.h"
#include "Calib.h"   // config.h + kalibrasi runtime
#include "EEMap.h"   // tata letak EEPROM + struct blok asing (satu definisi)
#include "types.h"
#include "HexaServos.h"
#include "HexaGait.h"
#include "ArmInverse.h"
#include "HexaArm.h"
#include "LegInverseKinematics.h"

class Hexapod {
public:
    Hexapod();
    void begin();
    void update();                       // panggil tiap loop

    // Gerak: maju(+)/mundur(-), geser kanan(+)/kiri(-), putar CCW(+)/CW(-). -1..1
    void walk(float forward, float strafe = 0.0f, float turn = 0.0f);
    void stop();

    // Stabilisasi badan dari IMU (derajat). Di-clamp + deadband + smooth.
    // void setStabilization(float rollDeg, float pitchDeg);

    // Pose badan manual (derajat & mm) -- untuk menunduk/menjinjit dsb.
    // Konvensi: +X kanan, +Y depan, +Z atas.
    //   roll  + = miring KANAN   (putar thd sumbu depan +Y)
    //   pitch + = MENDONGAK      (putar thd sumbu kanan +X)
    //   yaw   + = belok KIRI/CCW (putar thd +Z)
    // Nilai di-clamp ke BODY_MAX_ROT_DEG / BODY_MAX_TRANS_MM, lalu DI-RAMP
    // menuju target dengan laju BODY_SLEW_DEG_S / BODY_SLEW_MM_S. Keduanya
    // hanya menetapkan sasaran; yang menggerakkan adalah update().
    void setBodyTranslation(float x, float y, float z);
    void setBodyRotation(float rollDeg, float pitchDeg, float yawDeg);

    // Pose badan yang sedang BERLAKU (derajat & mm) -- hasil ramp, bukan
    // perintah terakhir. Selama ramp berjalan, angka ini ada di tengah jalan.
    float bodyRollDeg()  const { return rad2deg(_roll);  }
    float bodyPitchDeg() const { return rad2deg(_pitch); }
    float bodyYawDeg()   const { return rad2deg(_yaw);   }
    Vec3  bodyTranslation() const { return _trans; }

    // Pose badan yang DIMINTA (sesudah clamp). Bandingkan dengan yang di atas
    // untuk tahu apakah ramp masih berjalan.
    Vec3 bodyRotTargetDeg() const { return { rad2deg(_rollT), rad2deg(_pitchT), rad2deg(_yawT) }; }
    Vec3 bodyTransTarget()  const { return _transT; }
    bool bodyPoseSampai()   const;   // sudah tiba di target?

    // Sudut geometris hasil IK satu kaki pada pose badan yang berlaku sekarang.
    // Untuk telemetri & uji otomatis. return false bila di luar jangkauan.
    bool legAngles(int leg, float& coxaDeg, float& femurDeg, float& tibiaDeg);

    // false bila pose terakhir TIDAK bisa dituruti -- entah karena target di
    // luar jangkauan IK, atau karena sudut servo keluar dari 0..180 der.
    // Keduanya berarti sudut sudah di-clamp dan robot tidak menuruti perintah.
    bool lastPoseInRange() const { return !_ikClamped; }

    // Profil gait medan.
    void profileFlat();
    void profileStairs();
    void profileCrouch();
    void profileNarrow();
    void setGaitProfile(const GaitProfile& p) { _gait.setProfile(p); }

    // Profil yang BERLAKU sekarang -- hasil ramp, bukan yang terakhir diminta.
    // Selama transisi angkanya ada di tengah jalan; itu yang membuatnya
    // berguna, baik untuk 'T' maupun untuk uji.
    GaitProfile gaitProfile() const { return _gait.profile(); }

    // Odometri gait. Lihat HexaGait untuk arti dan batasnya.
    float lajuCms() const { return _gait.lajuMmS() * 0.1f; }
    float jarakCm() const      { return _gait.jarakMm() * 0.1f; }
    void  jarakNol()           { _gait.jarakNol(); }
    void  setSkalaOdo(float s) { _gait.setSkalaOdo(s); }
    float skalaOdo() const     { return _gait.skalaOdo(); }

    HexaArm* armDepan()    { return &_armF; }
    HexaArm* armBelakang() { return &_armB; }

    // --- Lengan (BAHU, SIKU, GRIP) ---
    // arm = ARM_DEPAN / ARM_BELAKANG.
    // jangkauan = jarak dari PUSAT BADAN ke arah hadap lengan itu (mm,
    //             positif = menjauh dari badan; lengan belakang memakai
    //             angka positif juga, artinya "sekian mm di belakang").
    // tinggi    = tinggi dari bidang pusat badan (mm, positif = ke atas).
    // Offset pangkal bahu dikurangkan di dalam fungsi.
    // return false bila target di luar jangkauan -- sudut TIDAK dikirim.
    bool moveArmTarget(uint8_t arm, float jangkauan, float tinggi);

    // Penjepit: 0 = menutup penuh, 100 = membuka penuh.
    bool setGrip(uint8_t arm, float persen);

    // Servo lengan default MATI (lengan belum tentu terpasang).
    void armEnable(uint8_t arm, bool on);
    bool armEnabled(uint8_t arm);

    // --- Keselamatan: robot boot dalam keadaan DISARMED (PWM mati) ---
    void arm();                          // hitung pose berdiri, lalu hidupkan servo
    void disarm();                       // matikan semua PWM, servo bebas
    bool isArmed() const { return _servos.isEnabled(); }

    // Cetak isi kalibrasi + hasil IK per kaki ke Serial (perintah 'd').
    void debugDump();

    // Tuner: gerak mentah 1 servo (id 0..NUM_TUNE_SERVOS-1, lihat TUNE_PIN_MAP). Langsung, tanpa gait.
    void jog(uint8_t tuneId, uint16_t pulseUs);

private:
    HexaServos _servos;
    HexaGait   _gait;
    HexaArm    _armF;   // lengan DEPAN    (slot kalibrasi 18,19,20)
    HexaArm    _armB;   // lengan BELAKANG (slot kalibrasi 21,22,23)

    // Pose badan BERLAKU (radian, mm) -- yang benar-benar dipakai IK.
    float _roll, _pitch, _yaw;
    Vec3  _trans;
    // Pose badan DIMINTA. _roll dst. merayap ke sini dengan laju terbatas.
    float _rollT, _pitchT, _yawT;
    Vec3  _transT;
    uint32_t _lastPoseT;   // stempel waktu untuk dt ramp pose badan

    // Offset Z per kaki dari EEPROM 2048 (Kalibrasi TES_GERAK)
    float _zOff[6] = {0, 0, 0, 0, 0, 0};
    bool  _zOffValid = false;
    bool  _mapLoaded = false;   // invert/trim berhasil diambil dari EEPROM 1024?
    bool  _ikClamped = false;    // pose terakhir tidak bisa dituruti (IK atau servo mentok)
    bool  _servoClamped = false; // diisi angleToPulse(): sudut servo keluar 0..180

    void loadServoMap();        // impor invert & trim hasil kalibrasi fisik
    void loadZOff();            // impor offset telapak (dgn cek checksum)
    void slewBodyPose();        // rayapkan pose berlaku menuju pose diminta

    // Satu kaki: gait -> transform badan -> frame kaki -> IK.
    // Dipakai bersama solvePose() & debugDump() supaya angka yang dicetak
    // dijamin identik dengan yang dikirim ke servo.
    // return false bila target kaki ini di luar jangkauan (sudut di-clamp).
    bool legSolve(int leg, float& lx, float& ly, float& lz,
                  float& coxa, float& femur, float& tibia);
    void solvePose();
    uint16_t angleToPulse(uint8_t servoID, float geoAngleDeg, float baseline);
};

#endif
