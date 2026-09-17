#ifndef HEXAGAIT_H
#define HEXAGAIT_H

// Generator tripod gait (mulus).
//  - stance: kaki menapak, geser LURUS (kecepatan konstan) -> tak slip.
//  - swing : trajektori SIKLOID (kecepatan nol di liftoff & touchdown) -> tak menyentak.
//  - YAW   : tiap kaki ikut komponen rotasi (cross product) -> bisa belok.
//  - vektor gerak di-SLEW (ramp) -> start/stop/belok mulus, sekaligus ease-in.
//  - profil medan di-RAMP (interpolasi) -> ganti medan tanpa badan melonjak.
//  Semua berbasis waktu (dt), tidak tergantung kecepatan loop.
//
// Output -> legTargets[6] (Vec3) di FRAME BADAN, dibaca oleh Hexapod.
#include "Calib.h"   // GAIT_* runtime
#include "types.h"

struct GaitProfile {
    float stepHeight;   // mm
    float stepLength;   // mm
    float cycleTime;    // ms
    float standHeight;  // mm (foot z = -standHeight)
    float standRadius;
};

class HexaGait {
public:
    HexaGait();
    void begin();
    void update();                       // hitung legTargets
    void setMoveVector(float vx, float vy, float vyaw); // -1..1 strafe, maju, putar
    // MENGHAPUS offset kaki. Profil biasa berlaku untuk KEENAM kaki sama
    // rata, jadi memasang profil apa pun berarti "kembali ke bentuk seragam".
    // Satu aturan, dan tidak ada profil yang bisa lupa membersihkannya --
    // yang butuh bentuk tak seragam memanggil setOffsetKaki() SESUDAH ini.
    void setProfile(const GaitProfile& p) {
        _tgtProf = p;
        for (int i = 0; i < 6; i++) _tgtOff[i] = { 0.0f, 0.0f, 0.0f };
    }

    // OFFSET PER KAKI, mm, di frame BADAN (+y depan, +z atas), ditambahkan ke
    // posisi netral kaki. Inilah satu-satunya cara menyatakan bentuk yang
    // TIDAK seragam: standHeight dan standRadius masing-masing satu angka
    // untuk enam kaki, jadi "kaki belakang naik, kaki depan maju" tidak bisa
    // ditulis dengan keduanya berapa pun nilainya.
    //
    // Di-ramp dengan tau yang sama dengan profil, dan ikut dihitung
    // profilTenang() -- jadi tunggu LiDAR di Misi otomatis ikut menunggunya.
    void setOffsetKaki(const Vec3 off[6]) {
        for (int i = 0; i < 6; i++) _tgtOff[i] = off[i];
    }

    // KOLOM PROFIL, berurutan sama dengan isi struct GaitProfile.
    enum KolomProfil : uint8_t {
        KOL_TINGGI_LANGKAH = 0,
        KOL_PANJANG_LANGKAH,
        KOL_WAKTU_SIKLUS,
        KOL_TINGGI_BADAN,
        KOL_RADIUS_KAKI,
        N_KOL_PROFIL
    };

    // PENYETELAN SATU KOLOM PROFIL, TANPA menyentuh offset kaki.
    //
    // setProfile() menghapus _tgtOff, dan itu memang BENAR untuk PEMASANGAN
    // profil: profil berlaku untuk keenam kaki sama rata, jadi memasang profil
    // apa pun berarti "kembali ke bentuk seragam". Tapi 'Th60' saat profil KAIL
    // aktif BUKAN pemasangan profil -- ia penyetelan satu angka pada bentuk
    // yang sedang dipakai. Memakai setProfile() di sana akan meratakan bentuk
    // KAIL diam-diam, dan bentuk itu justru yang membuat R-9 bisa dinaiki.
    //
    // Invarian "satu pintu pemasangan profil" TIDAK dilanggar: ini bukan pintu
    // kedua, karena ia tidak pernah memasang profil.
    //
    // Yang disetel adalah TARGET (_tgtProf), bukan yang sedang berlaku.
    // Mengetik 'Th60' di tengah transisi profil harus berarti "tinggi langkah
    // 60", bukan "bekukan separuh nilai ramp yang kebetulan sedang lewat".
    //
    // Nilai TIDAK diperiksa di sini: rentangnya milik Hexapod::setKolomProfil(),
    // supaya batasnya cuma ada di satu tempat bersama profilSah().
    bool setKolomProfil(uint8_t kolom, float nilai) {
        switch (kolom) {
            case KOL_TINGGI_LANGKAH:  _tgtProf.stepHeight  = nilai; return true;
            case KOL_PANJANG_LANGKAH: _tgtProf.stepLength  = nilai; return true;
            case KOL_WAKTU_SIKLUS:    _tgtProf.cycleTime   = nilai; return true;
            case KOL_TINGGI_BADAN:    _tgtProf.standHeight = nilai; return true;
            case KOL_RADIUS_KAKI:     _tgtProf.standRadius = nilai; return true;
            default: return false;
        }
    }
    GaitProfile profile() const { return _prof; }
    GaitProfile targetProfile() const { return _tgtProf; }

    // Posisi NETRAL telapak (frame badan), sesudah profil DAN offset per kaki
    // ikut di-ramp. Dipakai Hexapod untuk mengambil sudut lutut pose berdiri
    // saat lutut dikunci -- dengan begitu bentuk BERDIRInya tetap persis dan
    // hanya ayunannya yang berubah.
    Vec3 footHome(int i) const { return _footHome[i]; }

    // PROFIL SUDAH SAMPAI DI TARGETNYA? Ramp lerpf tidak pernah benar-benar
    // mendarat, jadi yang ditanya bukan "sama persis" melainkan "selisihnya
    // sudah di bawah 1 mm" -- di bawah itu badan tidak lagi bergerak sejauh
    // yang bisa dilihat sensor mana pun.
    //
    // cycleTime SENGAJA tidak ikut diperiksa: satuannya milidetik, bukan
    // milimeter, jadi satu ambang tidak bisa mengurus keduanya -- dan ia
    // TIMING, tidak memindahkan badan sedikit pun.
    //
    // Yang membutuhkannya: Misi, sebelum membaca LiDAR di ruas yang baru saja
    // berganti profil. Selama badan turun/naik, berkas sensor ikut berayun.
    bool profilTenang() const {
        for (int i = 0; i < 6; i++) {
            if (fabsf(_off[i].x - _tgtOff[i].x) >= 1.0f) return false;
            if (fabsf(_off[i].y - _tgtOff[i].y) >= 1.0f) return false;
            if (fabsf(_off[i].z - _tgtOff[i].z) >= 1.0f) return false;
        }
        return fabsf(_prof.standHeight - _tgtProf.standHeight) < 1.0f &&
               fabsf(_prof.standRadius - _tgtProf.standRadius) < 1.0f &&
               fabsf(_prof.stepHeight  - _tgtProf.stepHeight)  < 1.0f &&
               fabsf(_prof.stepLength  - _tgtProf.stepLength)  < 1.0f;
    }

    // ODOMETRI. Jarak bertanda yang sudah ditempuh badan sejak jarakNol();
    // mundur mengurangi. Dihitung dari fase gait dan vektor gerak yang sudah
    // di-slew DAN dinormalisasi, jadi ia mengukur apa yang benar-benar
    // dilakukan kaki -- bukan apa yang diperintahkan.
    float jarakMm() const { return _jarakMm; }

    // Laju maju sesaat, mm/detik, bertanda. Memakai skala slip yang sama
    // dengan odometer, jadi ia ikut terkalibrasi oleh pengukuran meteran.
    float lajuMmS() const { return _lajuMmS; }
    // GESER: jarak lateral bertanda, kanan(+)/kiri(-). Sumbu ini sudah ada di
    // setMoveVector() sejak awal tapi tidak pernah diakumulasi, jadi tidak ada
    // satu pun cara mengetahui robot sudah bergeser berapa jauh.
    float geserMm() const { return _geserMm; }

    // LINTASAN: panjang jejak yang sudah ditempuh telapak, SELALU BERTAMBAH.
    // Inilah yang layak dipakai rem jarak, dan bukan _jarakMm: yang terakhir
    // itu bertanda, jadi saat MUNDUR ia mengecil dan perbandingan
    // "jarak >= sasaran" tidak akan pernah benar -- robot mundur tanpa henti.
    // Geser murni juga tidak menambahnya sama sekali. Keduanya sudah terbukti
    // di lantai, 6 September 2026.
    float lintasMm() const { return _lintasMm; }

    void  jarakNol()      { _jarakMm = _geserMm = _lintasMm = 0.0f; }

    // Faktor slip. Dikalikan pada tiap PENAMBAHAN, bukan saat dibaca, supaya
    // menyetelnya di tengah jalan tidak menulis ulang jarak yang sudah
    // terkumpul. Hanya di RAM: menambah parameter Calib akan menaikkan
    // CALIB_VERSION dan membuang seluruh gain yang sudah disetel.
    void  setSkalaOdo(float s) { _skalaOdo = clampf(s, 0.5f, 1.5f); }
    float skalaOdo() const     { return _skalaOdo; }

    Vec3 legTargets[6];

private:
    GaitProfile _prof, _tgtProf;     // profil aktif (di-ramp) & target
    Vec3 _off[6], _tgtOff[6];        // offset per kaki (di-ramp) & target
    Vec3 _footHome[6];               // posisi netral ujung kaki (frame badan)
    float _tgtX, _tgtY, _tgtYaw;     // vektor gerak target
    float _curX, _curY, _curYaw;     // vektor gerak aktual (di-slew)
    bool _running;
    float _phase;
    float _jarakMm  = 0.0f;   // odometri maju, mm, bertanda
    float _geserMm  = 0.0f;   // odometri geser, mm, bertanda (kanan +)
    float _lintasMm = 0.0f;   // panjang lintasan, mm, selalu bertambah
    float _lajuMmS  = 0.0f;   // laju maju sesaat, mm/detik
    float _skalaOdo = ODO_SKALA_DEF;   // koreksi slip; diukur 1,0, ditimpa 'Ds'
    unsigned long _lastUpdate;
    void computeHome();
    float dtSeconds();
};

#endif
