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

    // LAJU RAMP GESER BADAN, mm/detik. Baku BODY_SLEW_MM_S; sekuens korban
    // memperlambatnya untuk condong maju, yang dikerjakan tepat di atas
    // korban. Disetel per gerakan, bukan selamanya -- yang menyetelnya WAJIB
    // mengembalikannya, kalau tidak stabilisasi IMU ikut lambat tanpa sebab
    // yang kelihatan. Pola yang sama dengan HexaArm::setSlew().
    void  setBodySlewMm(float mmS) { _bodySlewMm = (mmS > 1.0f) ? mmS : 1.0f; }
    float bodySlewMm() const       { return _bodySlewMm; }

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
    void profileKail();          // R-9: depan mengait, belakang mengangkat hidung
    void profileTanjak();        // R-9 lambat: langkah tinggi, kaki depan maju
    void setGaitProfile(const GaitProfile& p) { pasangProfil(p); }
    // Dipakai profileKail() dan uji jangkauan; profil biasa menghapusnya
    // sendiri lewat setProfile().
    void setOffsetKaki(const Vec3 off[6]) { _gait.setOffsetKaki(off); }

    // Profil yang BERLAKU sekarang -- hasil ramp, bukan yang terakhir diminta.
    // Selama transisi angkanya ada di tengah jalan; itu yang membuatnya
    // berguna, baik untuk 'T' maupun untuk uji.
    GaitProfile gaitProfile() const { return _gait.profile(); }

    // Ramp profil sudah selesai? Lihat HexaGait::profilTenang().
    bool gaitProfilTenang() const { return _gait.profilTenang(); }

    // Odometri gait. Lihat HexaGait untuk arti dan batasnya.
    float lajuCms() const { return _gait.lajuMmS() * 0.1f; }
    float jarakCm() const      { return _gait.jarakMm() * 0.1f; }
    float geserCm() const      { return _gait.geserMm() * 0.1f; }
    float lintasCm() const     { return _gait.lintasMm() * 0.1f; }
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
    // HANYA ARM_DEPAN; ARM_BELAKANG cuma punya grip dan selalu return false.
    bool moveArmTarget(uint8_t arm, float jangkauan, float tinggi);

    // Sama, tapi (jangkauan, tinggi) menunjuk TITIK CAPIT, bukan pergelangan,
    // dan pergelangan ikut disetel supaya TAPAK MENDATAR. Karena tapaknya
    // mendatar, capit selalu HAND_LENGTH mm lurus di depan pergelangan --
    // itulah yang membuat satu pasang angka bisa berarti "di mana capitnya".
    //
    // Inilah yang dipakai sekuens korban. moveArmTarget() menaruh PERGELANGAN
    // di titik yang diminta, dan memakainya untuk membidik korban berarti tiap
    // pemanggil harus mengurangkan HAND_LENGTH sendiri -- 120 mm yang cuma
    // perlu lupa sekali.
    //
    // Lebih ketat daripada moveArmTarget(): ia juga MENOLAK pose yang sudut
    // servonya keluar 0..180 atau yang pergelangannya tak sampai. Untuk lengan
    // itu penting -- angleToPulse() meng-clamp DIAM-DIAM, tanpa penanda
    // seperti _servoClamped milik kaki, jadi tanpa pemeriksaan ini sekuens
    // akan "berhasil" sambil menaruh capit di tempat yang salah.
    // tapakDeg = sudut TAPAK terhadap mendatar, derajat; + = menengadah,
    // - = menunduk. 0 (baku) = mendatar ke depan.
    //
    // Dengan pergelangan ikut ditentukan, lengan depan menjadi 3 sendi di
    // bidang 2 dimensi, dan pose bidang itu memang 3 angka: (x, y, sudut).
    // Jadi ketiganya bisa dipenuhi PERSIS, tanpa sisa kebebasan -- tidak ada
    // pilihan yang harus ditebak. Rumusnya: mundur HAND_LENGTH dari titik
    // capit searah tapak untuk mendapat titik pergelangan, pecahkan 2 link ke
    // situ, lalu pergelangan = tapak - (bahu + siku).
    //
    // Sudut tapak MELEBARKAN amplop: menunduk membuat capit bisa turun lebih
    // dekat ke badan daripada yang sanggup dicapai tapak mendatar. Lihat
    // sapuannya di cek_korban.cpp sebelum memakainya -- yang melebar bukan
    // jangkauan lengan, melainkan bagian jangkauan yang lolos batas servo.
    bool moveArmGrip(uint8_t arm, float jangkauan, float tinggi, float tapakDeg = 0.0f);

    // KETIGA SENDI LANGSUNG, TANPA IK. Untuk membidik pose dengan tangan lalu
    // menuliskannya keras -- IK menjawab "sudut mana yang menaruh capit di
    // sini", perintah ini menjawab pertanyaan sebaliknya.
    //
    // Sudutnya GEOMETRIS, satuan yang sama dengan IK dan setPergelangan(),
    // BUKAN sudut servo 0..180. Jalurnya pun sama persis (baseline + offset +
    // invert + trim), jadi pose yang ditemukan lewat perintah ini berulang
    // sama lewat moveArmGrip() -- itu gunanya di-hard-code.
    //
    // Sudut TETAP DIKIRIM walau return false. angleToPulse() meng-clamp
    // diam-diam, dan menolak seluruh pose justru menyembunyikan batas yang
    // sedang dicari; yang dibutuhkan bukan penolakan melainkan pemberitahuan.
    // servoOut (3 angka, boleh nullptr) diisi sudut servo SEBELUM clamp,
    // supaya pemanggil bisa mencetak sendi mana yang mentok dan seberapa.
    bool setSudutLengan(uint8_t arm, float bahuDeg, float sikuDeg, float prgDeg,
                        float* servoOut = nullptr);

    // Pergelangan lengan depan: sudut ketiga, -90..+90 der dari baseline.
    // Disetel sendiri, TIDAK diturunkan dari bahu+siku. false untuk belakang.
    bool setPergelangan(uint8_t arm, float deg);

    // Laju slew lengan, derajat/detik. Lihat HexaArm::setSlew() -- pemanggil
    // yang memperlambatnya wajib mengembalikannya ke ARM_SLEW_DEG_S.
    void setSlewLengan(uint8_t arm, float degS);

    // Grip bukan id servo yang sama di kedua lengan -- depan servo ke-4,
    // belakang satu-satunya servo. Jangan menulis angkanya langsung.
    static uint8_t gripId(uint8_t arm) {
        return (arm == ARM_DEPAN) ? ARM_ID_GRIP_DEPAN : ARM_ID_GRIP_BELAKANG;
    }

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
    float    _bodySlewMm = BODY_SLEW_MM_S;   // laju ramp geser badan, mm/detik

    // Offset Z per kaki dari EEPROM 2048 (Kalibrasi TES_GERAK)
    float _zOff[6] = {0, 0, 0, 0, 0, 0};
    bool  _zOffValid = false;
    bool  _mapLoaded = false;   // invert/trim berhasil diambil dari EEPROM 1024?
    bool  _ikClamped = false;    // pose terakhir tidak bisa dituruti (IK atau servo mentok)
    bool  _servoClamped = false; // diisi angleToPulse(): sudut servo keluar 0..180

    void loadServoMap();        // impor invert & trim hasil kalibrasi fisik
    void loadZOff();            // impor offset telapak (dgn cek checksum)

public:
    // --- TRIM SERVO, disetel saat firmware misi sedang jalan ---------------
    //
    // Sebelum 18 Sep 2026 trim hanya bisa disetel dengan mem-flash sketsa lain
    // (legacy-2026/KALIBRASI). Artinya firmware misi hilang dari Teensy, lalu
    // harus di-flash balik -- dua flash, robot lemas dua kali. Di arena itu
    // waktu yang tidak ada.
    //
    // Sumber kebenarannya tetap EEPROM 1024, sama seperti dulu, jadi sketsa
    // legacy dan firmware ini membaca angka yang sama.
    void setTrim(uint8_t slot, int16_t us);   // RAM saja, belum tersimpan
    bool simpanServoMap();                    // RAM -> EEPROM 1024
    void cetakTrim();                         // tabel '#TRIM', dibaca HUD
    void nolkanTrim();                        // RAM saja

private:
    void slewBodyPose();        // rayapkan pose berlaku menuju pose diminta

    // Satu kaki: gait -> transform badan -> frame kaki -> IK.
    // Dipakai bersama solvePose() & debugDump() supaya angka yang dicetak
    // dijamin identik dengan yang dikirim ke servo.
    // return false bila target kaki ini di luar jangkauan (sudut di-clamp).
    bool legSolve(int leg, float& lx, float& ly, float& lz,
                  float& coxa, float& femur, float& tibia);
    // Sama, tapi untuk titik telapak SEMBARANG dan dengan lutut bisa dikunci.
    bool legSolveAt(int leg, const Vec3& foot, float& lx, float& ly, float& lz,
                    float& coxa, float& femur, float& tibia, float kunciLutut);
    // Sudut lutut kaki ini kalau dikunci; NAN = tidak dikunci. Diambil dari
    // pose NETRAL kaki, jadi bentuk berdirinya tetap persis.
    float kunciLutut(int leg);

    // SATU-SATUNYA pintu pemasangan profil. Menghapus kunci lutut, persis
    // seperti HexaGait::setProfile() menghapus offset kaki: profil yang tidak
    // memintanya tidak bisa lupa membersihkannya, dan kaki depan tidak
    // tertinggal beku sesudah keluar dari KAIL.
    void pasangProfil(const GaitProfile& p) { _lututKunci = 0; _gait.setProfile(p); }
    uint8_t _lututKunci = 0;     // bitmask kaki yang lututnya dibekukan

    void solvePose();
    uint16_t angleToPulse(uint8_t servoID, float geoAngleDeg, float baseline);
};

#endif
