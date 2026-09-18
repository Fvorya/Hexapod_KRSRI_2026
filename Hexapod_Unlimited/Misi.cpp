#include "Skor.h"

// Pembukuan poin milik .ino; di sini cuma diberi tahu ruas mana yang selesai.
extern Skor gSkor;
#include "Misi.h"

// ====================================================================
// TABEL LINTASAN -- Guidebook SAR UNLIMITED 2026, halaman 21-30.
//
// Urutan ruas mengikuti gambar "jalur misi robot" (hal. 21 & 34):
//   HOME -> K-1 -> R-1 -> R-2 -> R-3 -> R-4/SZ-1 -> K-2 -> R-5/SZ-2
//        -> R-6 -> K-3 -> K-4 -> R-7/R-8/SZ-3 -> R-9(tangga) -> R-10
//        -> SZ-4 -> K-5 -> R-11 -> SZ-5/FINISH
//
// ATURAN URUTAN (hal. 21): robot harus MENCOBA mengangkat K-1 sebelum
// melewati batas R1-R2. Kalau R1 sudah dilewati lalu robot kembali mengambil
// K-1, pengangkatannya TIDAK SAH. Karena itu ruas korban duduk di tempatnya
// di tabel ini, bukan dilompati -- walau capitnya belum ada.
//
// ARAH ditulis sebagai BELOK RELATIF dari ruas sebelumnya, bukan mata angin
// mutlak; mutlaknya dihitung hitungArah() dan bisa dilihat di 'm4'. Alasannya
// ada di Misi.h. Jangkarnya MISI_ARAH_BERANGKAT = 0, sama dengan
// MISI_ARAH_AWAL milik adik tingkat, jadi tiga ruas pertama menghasilkan
// UTARA / BARAT / UTARA -- persis angka yang sudah pernah dia jalankan.
//
// Lintasannya satu putaran U: lurus sepanjang baris bawah, belok kanan dua
// kali di ujung kanan (menyeberang lalu kembali), lurus sepanjang baris atas,
// belok kanan sekali lagi turun ke FINISH.
//
// TIKUNGANNYA WAJIB DICOCOKKAN DENGAN ARENA SAAT MAPPING -- ini turunan dari
// gambar guidebook, bukan dari pengukuran. Untungnya salah tikungan cuma
// perlu satu baris diubah: sisa lintasan menyesuaikan sendiri.
//
// NILAI: -1 = BELUM DIUKUR, misi menolak berangkat. Yang sudah terisi datang
// dari angka yang benar-benar tertulis di guidebook:
//   R-4  45x60      hal. 25      R-5  45x45 (lumpur)  hal. 25
//   R-6  45x55      hal. 26      R-9  M2 tangga 90 horizontal / 50 vertikal,
//   R-10 50 horizontal (50,2 miring)  hal. 29         anak tangga 2 x 3,6 cm
//   M1   80 horizontal / 20 vertikal = 14,0 der       hal. 22
// LINTASAN DIROMBAK 6 September 2026 (malam), dari ruas 4 ke bawah, atas
// arahan operator yang berdiri di arena. Yang lama dibaca dari gambar
// guidebook; yang ini dari mata. Bentuknya sekarang:
//
//   R-4 -> pivot BARAT (lewat K-2, TEMBUS R-5) -> pivot SELATAN saat sudah di
//   posisi mau keluar R-5 -> maju sedikit -> GESER KANAN -> SELATAN sampai
//   tembok K-3 -> pivot TIMUR sampai tembok -> pivot SELATAN lewat R-6 ->
//   pivot BARAT lalu SELATAN (sepasang pivot pelurus) -> R-8 ke kaki tangga
//   -> ratakan 20 cm ke dinding kanan -> TANGGA.
//
// PEMERIKSAAN YANG MEYAKINKAN, sama seperti perombakan sebelumnya: rantai
// belok yang baru tetap mendaratkan TANGGA di SELATAN, arah yang sudah
// diverifikasi di arena. Ada tujuh belokan di antara R-4 dan tangga; kalau
// satu saja keliru, tangganya tidak akan lagi jatuh di SELATAN. Jadi ada yang
// memeriksa pekerjaan ini selain penalaran.
//
// APA YANG HILANG BERSAMA PEROMBAKAN INI, dan ini kerugian nyata:
//   Ruas 8 lama -- "R-5 SELATAN ikut dinding kanan, 65 cm" -- adalah SATU-
//   SATUNYA ruas yang pernah lolos utuh di robot ('m4 8 8', odometri 85,5 ->
//   151,1 = 65,6 cm). R-5 sekarang ditembus ke BARAT, jadi 65,6 cm tidak
//   berlaku lagi: ia diukur sepanjang sumbu yang lain. Panjang tembusan BARAT
//   itu BELUM PERNAH DIUKUR.
//   Ruas 9 lama -- "geser TIMUR keluar R-5", yang gagal dua kali dengan
//   "halangan di depan" -- ikut hilang. Jog ke TIMUR sekarang duduk SESUDAH
//   K-3, bukan sesudah R-5.
//   K-4 tidak lagi punya baris sendiri; 10 cm-nya terserap ke ruas sekitarnya.
//   Alasan MELEWATINYA tidak berubah -- lihat blok KAPASITAS CAPIT di bawah.
//
// ANGKA YANG SUDAH DIUKUR:
//   ruas 0 (32), 2 (84), 3 (66), 4 (15), 6 (7), 7 (53), 9 (11), 11 (31),
//   12 (13), 14 (20), 15 (27), 18 (66), 20 (20), 21 (20), 24 (20),
//   26 (25), 27 (50), 29 (20), 25 (103, geometri).
//   Ketiga yang pertama DIUKUR ULANG 7 September 2026 dengan "m6" pada
//   jalur dan profil yang berlaku sekarang -- lihat catatan per ruas.
//
// DUA ANGKA MASIH -1 dan HARUS DIUKUR sebelum 'm1' penuh bisa berangkat:
//   ruas 23 (jalan ke depan tangga) dan 31 (R-11 longsor).
//   TABEL DIROMBAK OPERATOR 8 Sep 2026 (malam): 28 -> 34 baris. Pasangan
//   pivot pelurus dan ruas R-8 DIBUANG; SZ-3, K-4, dan empat ruas geser/
//   maju pendek ditambahkan. Nomor 19 ke atas semuanya bergeser -- catatan
//   di bawah sudah mengikuti tabel yang SEKARANG.
//   PENOMORAN BERGESER SEKALI LAGI 8 Sep 2026: DUA baris disisipkan sesudah
//   ruas 13 (geser KIRI, lalu maju ke samping K-3), jadi tiap nomor 14 ke
//   atas NAIK dua. Keduanya BLK_LURUS.
//   PENOMORAN BERGESER LAGI 7 Sep 2026 (malam): baris "SZ-2 taruh korban"
//   DISISIPKAN sesudah ruas 9, jadi tiap nomor 10 ke atas NAIK satu.
//   Beloknya BLK_LURUS, jadi tidak satu pun mata angin bergeser.
//   PENOMORAN BERGESER 15 Sep 2026: ruas "putar KIRI 30 der lalu maju"
//   DISISIPKAN sesudah ruas 6, jadi tiap nomor 7 ke atas NAIK satu. Label
//   /*N*/ di tabel sudah digeser; catatan per-ruas DI BAWAH INI masih memakai
//   nomor LAMA, dan sebagiannya memang sudah tidak sinkron sejak sebelum ini.
//   Tabel yang benar selalu 'm4'.
//
//   Ruas itu MENYERONG, dan serong menumpuk: ia menulis -30 dan ruas 8
//   menulis +30 supaya sisa lintasan kembali ke mata angin murni. Kedua
//   angka itu sepasang -- mengubah salah satunya sendirian memiringkan
//   seluruh sisa tabel.
//
//   KEMUDI-nya KMD_KANAN mengikuti tetangganya, dan itu BELUM DIUJI pada
//   badan yang miring 30 der: berkas LiDAR samping menempuh 1/cos(30) = 1,15
//   kali jarak tegak lurusnya, jadi ikut-dinding membaca lebih jauh daripada
//   yang sebenarnya. Kalau robot merapat ke dinding di ruas ini, KMD_TENGAH
//   tersangka penggantinya.
//
//   PENOMORAN BERGESER 7 Sep 2026: ruas 5 lama ("R-4: dekati dinding utara")
//   DIHAPUS, jadi tiap nomor 6 ke atas turun satu. Beloknya BLK_LURUS,
//   sehingga penghapusannya TIDAK menggeser satu pun mata angin.
//   Yang HNT_ODO diukur dengan 'm6 <idx>' lalu ditulis dengan 'm7 <idx> <cm>'.
//   Ruas 12 HNT_SISI TIDAK bisa diukur begitu -- ia jarak ke dinding samping,
//   bukan jarak tempuh; cobalah 'V<cm>' langsung lalu tulis angkanya.
//
// ====================================================================
// CATATAN PER RUAS. Nomor di sini nomor TABEL DI BAWAH, dan hanya sah selama
// tabelnya tidak dipecah lagi -- perombakan sebelumnya sempat meninggalkan
// seluruh blok ini tertinggal satu nomor dari tabelnya sendiri.
//
//   ruas 0  32 cm  DIKOREKSI dari 40 pada uji misi 6 Sep 2026 (sore): dengan
//                  40 robot berhenti 5-10 cm MELEWATI K-1, jadi ceruk korban
//                  sudah di belakangnya saat ia berhenti. Diambil 8 cm, tengah
//                  rentang yang dilaporkan operator.
//                  Angka 40 sebelumnya datang dari mapping manual, dan di situ
//                  yang dicocokkan adalah letak BUKAAN sisi kiri -- bukan titik
//                  berhenti yang benar untuk mengangkat korban. Bukaan mulai
//                  terbaca lebih awal daripada tempat robot harus berdiri.
//                  Catatan: nilai HNT_BELAKANG adalah JARAK TEMPUH yang diukur
//                  sensor belakang, bukan ambang bacaan. Log misi mencetaknya
//                  apa adanya: "titik nol 41,0 cm -> berhenti di bacaan 81,0".
//                  32 -> 34 pada pengukuran ulang 7 Sep 2026 (siang), lalu
//                  DIKEMBALIKAN ke 32 oleh operator pada trial 7 Sep (malam).
//                  PERHATIAN: "m6" mengembalikan ODOMETRI, sedangkan baris ini
//                  dijalankan dengan SENSOR BELAKANG. Keduanya mengukur jarak
//                  tempuh yang sama tapi tidak pernah sepakat persis -- 6 Sep
//                  odometri 41,5 lawan sensor belakang 39. Jadi 32 boleh meleset
//                  satu-dua cm dari apa yang sensor belakang lihat; kalau robot
//                  masih berhenti melewati K-1, itu sebabnya, bukan salah ukur.
//   ruas 2  84 cm  R-1. DIUKUR ULANG 7 Sep 2026 dengan "m6 2", profil TANGGA
//                  yang sama dengan yang dipakai menjalankannya. 83 yang lama
//                  datang dari mapping manual dengan rem jarak, dan pada uji
//                  "m4 0 3" robot kebablasan di ujung jalan pecah. Selisihnya
//                  cuma 1 cm, jadi ruas ini bukan sebab utama kebablasan itu;
//                  ruas 3 yang menyumbang hampir seluruhnya.
//                  Empat lompatan berpagar rem jarak. Dinding KANAN
//                  HILANG sepanjang bagian tengahnya, KIRI selalu ada ->
//                  kemudinya KIRI, bukan TENGAH.
//   ruas 3  66 cm  M1. DIUKUR ULANG 7 Sep 2026 dengan "m6 3": operator
//                  menghentikan robot begitu badan keluar dari bidang miring
//                  dan lantai kembali datar. 88 -> 60, selisih 28 cm, dan itu
//                  yang membawa robot hampir sampai ujung R-4 pada uji
//                  "m4 0 3" -- di ruas BUTA, tempat tidak ada apa pun yang
//                  menghentikannya.
//                  40 sempat dipertimbangkan (berhenti sebelum lantai datar)
//                  lalu DIBATALKAN: 60 menaruh robot di lantai datar, dan
//                  ruas berikutnya menyeberangi koral, dan gait TANGGA-nya
//                  menuntut badan sudah datar sebelum kaki mulai diangkat.
//                  DIPASTIKAN 7 Sep 2026: BATU R-4 MULAI PERSIS DI 60. Robot
//                  menuruni bidang miring lalu langsung masuk R-4 dengan badan
//                  sudah datar. Jadi batas ruas 3/4 jatuh tepat di tepi koral,
//                  dan pembagian profilnya benar dengan sendirinya: DATAR untuk
//                  seluruh turunan, TANGGA untuk seluruh hamparan batu. Tidak
//                  ada ruas yang menyeberangi batu dengan gait lantai rata.
//                  SEBABNYA TERLACAK: 88 diukur saat baris ini masih
//                  PRF_MERUNDUK (commit b9c425c). Profilnya kemudian diganti ke
//                  PRF_DATAR karena MERUNDUK berhenti di tengah turunan, TAPI
//                  angkanya ikut terbawa. Langkah DATAR lebih panjang dan
//                  selipnya di bidang 14 der berbeda, jadi 88 itu angka milik
//                  gait yang lain. Pelajarannya berlaku untuk seluruh tabel:
//                  MENGGANTI PROFIL MEMBATALKAN PANJANG RUASNYA.
//                  60 -> 66 pada trial 7 Sep 2026 (malam), dari operator.
//                  BELUM DICOCOKKAN dengan "BATU R-4 MULAI PERSIS DI 60" di
//                  atas: kalau 66 benar, batas ruas 3/4 jatuh 6 cm DI DALAM
//                  hamparan batu.
//                  Angka lama: jejak pitch IMU -15,1 -> +0,7 der, titik datar
//                  di ~88 menurut MERUNDUK.
//                  R-2 dan R-3 TERNYATA BERADA DI TURUNAN YANG SAMA, bukan
//                  lorong terpisah sesudahnya -- dua baris tabel dihapus.
//                  Guidebook 80 cm horizontal (miring 82,5); odometri membaca
//                  ~6 cm lebih panjang menuruni bidang miring.
//                  PROFIL MERUNDUK -> DATAR, 6 Sep 2026 (sore), atas
//                  pengamatan operator: dengan MERUNDUK robot BERHENTI DI
//                  TENGAH TURUNAN dan tidak pernah sampai lantai datar.
//                  MERUNDUK dipilih dulu dari penalaran, bukan percobaan:
//                  badan turun 20 mm menurunkan titik berat, dan langkah
//                  dipendekkan 60 -> 45 mm supaya kaki tidak menggantung jauh
//                  di bibir turunan. Penalarannya masih masuk akal, tapi ARENA
//                  MENOLAKNYA. Kalau nanti ada yang mau mencoba MERUNDUK lagi,
//                  naikkan dulu panjang langkahnya, jangan tinggi badannya.
//                  Ruas 26 (R-10) sejak 8 Sep PRF_DATAR, bukan MERUNDUK --
//                  ia bidang miring juga, jadi curigai hal yang sama di sana.
// NOMOR RUAS DI CATATAN DI BAWAH INI SUDAH TIDAK SINKRON dengan tabelnya.
// Tabel dirombak dari arena beberapa kali (34 -> 32 -> 33 baris) dan catatan
// per-ruas tidak ikut dinomori ulang. Isinya masih benar; yang basi cuma
// nomornya. Cari berdasarkan NAMA ruas, bukan angkanya.
//
//   ruas 4  15 cm  SESUDAH TURUNAN, MAJU KE TEMBOK. DITAMBAHKAN 9 Sep 2026
//                  atas permintaan operator. Gunanya satu: menyerap hanyutan
//                  odometri ruas 3, yang menempuh 88 cm BUTA melewati turunan
//                  M1. Berapa pun melesetnya lompatan itu, ruas ini berakhir
//                  di jarak yang SAMA dari tembok, jadi pivot kiri ruas 5
//                  berangkat dari titik yang tetap.
//                  Ambangnya 15, bukan 21: FRONT_STOP_CM turun 20 -> 12 pada
//                  hari yang sama, dan itulah yang membuat baris seperti ini
//                  mungkin sama sekali. Kalau robot berhenti terlalu jauh,
//                  turunkan ke 13 ("m7 4 13") -- 13 adalah batas bawahnya,
//                  satu di atas FRONT_STOP_CM.
//                  TIDAK BUTA, dan tidak boleh: seluruh gunanya sensor depan.
//                  BELOKNYA BLK_LURUS, jadi menyisipkannya tidak menggeser
//                  satu pun mata angin sesudahnya -- hanya nomornya.
//   ruas 4  16 cm  R-4 SELURUHNYA, dari tepi batu sampai SZ-1. DIUKUR 7 Sep
//                  2026: "m6 4" mencetak 16,2 cm saat operator berhenti tepat
//                  sebelum SZ-1, dengan LiDAR depan membaca 21-22 cm. Jadi sisa
//                  R-4 dari tepi batu ke dinding utara hanya ~37,7 cm, dan SZ-1
//                  (20x20) memenuhi hampir seluruh 21,5 cm terakhirnya.
//
//                  R-4 DULU DIPECAH DUA, dan pemecahan itu DIBATALKAN 7 Sep 2026
//                  atas pengamatan operator. Baris keduanya ("R-4: dekati dinding
//                  utara", HNT_DEPAN 25) tidak sanggup dipertahankan:
//                  1. Ia cuma menempuh ~3 cm. Satu baris tabel untuk tiga
//                     sentimeter.
//                  2. Ambang 25 memberhentikan robot ~3,5 cm DI BELAKANG titik
//                     yang dinilai benar operator, dan dari situ kaki belakang
//                     menyerempet dinding BARAT saat ruas 6 menyeberang ke K-2.
//                  3. Ia salah satu baris HNT_DEPAN yang dicurigai kena hantu
//                     sensor depan, dan 25 cm cuma 5 cm di atas FRONT_STOP_CM saat itu (20) --
//                     navigasi bisa mengerem sendiri sebelum misi sempat.
//                  Alasan pemecahannya dulu ada dua, dan keduanya gugur: "ruas 5
//                  menyalakan sensor depan di lantai yang sudah tenang" KELIRU
//                  (seluruh R-4 tertutup koral, tidak ada lantai bersih), dan
//                  "menyerap hanyutan odometri" tidak sebanding untuk lintasan
//                  sependek 16 cm.
//                  YANG HILANG BERSAMANYA: tidak ada lagi sensor yang menghentikan
//                  robot sebelum dinding utara kalau odometri meleset ke depan.
//                  Pagarnya sekarang cuma panjang ruas ini. Kalau robot berhenti
//                  terlalu jauh atau kurang, setel dengan "m7 4 <cm>" -- satu angka,
//                  tanpa ambang sensor yang melawannya.
//                  UTANG: begitu capit terpasang, 16 cm menaruh robot tepat di
//                  tepi SZ-1 dan jangkauan lengan ~12 cm dari pusat badan mungkin
//                  masih kurang. Jalan keluarnya ruas geser atau ruas odometri
//                  pendek TAMBAHAN, bukan menghidupkan lagi ambang sensor depan
//                  di dekat FRONT_STOP_CM.
//   ruas 6   7 cm  MAJU SEDIKIT sebelum pivot BARAT. DITAMBAHKAN 7 Sep 2026
//                  dari arena: robot berhenti di depan SZ-1 pada posisi yang
//                  BENAR untuk menaruh korban, tapi begitu memutar ke BARAT dan
//                  berjalan, ia MENABRAK dinding di dekat K-2. Titik yang benar
//                  untuk menaruh korban bukan titik yang benar untuk berangkat
//                  menyeberang, dan satu ruas tidak bisa jadi keduanya.
//                  Pola yang sama sudah ada di ruas 11 ("keluar R-5, maju
//                  dikit"): ruas pembebas pendek antara aksi di tempat dan
//                  tikungan berikutnya.
//                  BUTA WAJIB, dan ini bukan pilihan gaya: dinding utara tinggal
//                  ~21,5 cm di depan sesudah ruas 4, jadi maju beberapa cm saja
//                  menjatuhkan bacaan depan di bawah FRONT_STOP_CM (dulu 20, kini 12) dan
//                  navigasi mengerem "halangan di depan" sebelum ruas ini sempat
//                  menempuh apa pun. Buta menuntut HNT_ODO -- terpenuhi.
//                  PROFIL TANGGA: masih di atas koral R-4.
//                  BELOKNYA BLK_LURUS, jadi menyisipkannya TIDAK menggeser satu
//                  pun mata angin ruas sesudahnya -- hanya nomornya.
//                  7 cm dari operator, trial 7 Sep 2026 (malam). Ruas ini BUTA
//                  dan pendek: yang menghentikannya cuma odometri, sedangkan
//                  dinding utara tinggal ~21,5 cm di depan. Kalau kaki
//                  menyentuh dinding, ukur ulang dengan "m6 6".
//   ruas 7  53 cm  menyeberang ke K-2, tiga lompatan berpagar rem lintasan.
//                  44 -> 53 pada trial 7 Sep 2026 (malam), dari operator.
//                  BELOKNYA KIRI, bukan KANAN: K-2 ada di sisi BARAT, dan
//                  ceruknya terbaca jelas -- kedua sensor kiri terbuka
//                  bersamaan (9 -> 29 cm) lalu menutup lagi sesudah dilewati.
//   ruas 9         R-5 BUKAN cabang: robot sampai di K-2 lalu mendapati
//                  dirinya SUDAH di dalam R-5 tanpa pernah berbelok, jadi
//                  beloknya LURUS. Diamati 6 Sep 2026. Yang berubah malam itu
//                  hanya SUMBUNYA: ditembus ke BARAT sampai ujung, bukan
//                  dibelokkan ke SELATAN di K-2.
//                  Kelerengnya terasa di odometri saat ditempuh ke selatan:
//                  satu lompatan butuh 7,2 detik untuk jarak yang tadi 4,3
//                  detik, dan sensor depan cuma turun 10 cm untuk 15 cm tempuh
//                  (selip). Profil TANGGA dipertahankan karena itu.
//   ruas 10        SZ-2 taruh korban K-2. DISISIPKAN 7 Sep 2026 (malam) atas
//                  permintaan operator. HNT_LANGSUNG: ruas aksi di tempat,
//                  tidak menempuh jarak, jadi tidak ada yang perlu diukur.
//                  BLK_LURUS supaya penyisipannya tidak menggeser mata angin.
//                  TINGGI LEPAS BEDA: SZ-2 duduk 4 cm DI ATAS lantai (hal. 25,
//                  28), tidak sejajar lantai seperti SZ-1/SZ-3. Baru berarti
//                  begitu capit terpasang; sekarang ia berhenti kosong.
//                  SELESAI 8 Sep: K-2 lepas di sini sempat membuat SZ-4
//                  kehilangan korbannya, dan tabelSiap() menolak berangkat.
//                  Ditutup dengan menambahkan K-4 (ruas 22). Rantai capit
//                  kini berselang rapi: 1-5, 8-10, 16-19, 22-28, 30-33.
//   ruas 11        "maju sedikit" sesudah pivot SELATAN. Sengaja tidak
//                  ditebak: menebak panjang ruas berarti mengganti profil gait
//                  di tempat yang salah, dan di sini tempat yang salah itu
//                  bibir R-5.
//   ruas 12        GESER KANAN, ruas HNT_SISI pertama di tabel ini. Sumbu
//                  geser tidak bisa diminta dari luar -- perpindahannya
//                  terkuantisasi satu langkah gait penuh, dan dua perintah
//                  IDENTIK pernah berbeda tiga kali lipat (+3 lalu +9 cm).
//                  Yang membuatnya bisa dipakai adalah syarat henti yang
//                  dibaca TIAP TICK di dalam firmware; lihat
//                  Navigation::ratakanMulai(). Sasarannya masih -1: operator
//                  belum menyebut mau berapa cm dari dinding kanan.
//   ruas 13 25 cm  SELATAN sampai tembok K-3. HNT_DEPAN -- baca PERINGATAN
//                  SENSOR DEPAN di bawah sebelum menjalankannya.
//   ruas 14,15     MENDEKATI K-3, DITAMBAHKAN 8 Sep 2026 dari arena: geser
//                  KIRI sedikit, lalu maju sampai badan berada DI SAMPING
//                  korban. Dua baris, bukan satu: HNT_SISI hanya menggeser
//                  menyamping dan tidak bisa sekaligus maju.
//                  Ruas 14 HNT_SISI -> nilainya JARAK KE DINDING KIRI, bukan
//                  jarak tempuh, jadi "m6" tidak berlaku; coba "V<cm>" lalu
//                  tulis angkanya. Sasarannya WAJIB di atas wall.min (12) --
//                  kalau tidak, ratakanMulai() menolaknya sebelum berangkat.
//                  Ruas 15 HNT_ODO biasa -> "m6 15" lalu "m7 15 <cm>".
//                  KEMUDI KIRI di kedua baris: sesudah sengaja menggeser ke
//                  kiri, mengikuti dinding KANAN akan menyeret robot kembali
//                  ke kanan dan membatalkan gesernya. Kalau di arena dinding
//                  kiri ternyata hilang di sini, tukar ke KMD_KANAN.
//   ruas 16        K-3 angkat korban. BELOKNYA BLK_KANAN sejak 8 Sep 2026:
//                  badan pivot ke BARAT dulu, baru mengangkat. Sebelumnya
//                  baris ini BLK_LURUS (tetap SELATAN).
//                  Lengan BELAKANG cuma grip -- tidak ada sendi yang bisa
//                  mengoreksi, jadi ruas 14 dan 15 yang harus menaruh badan
//                  tepat. Korban ini nantinya ditaruh di SZ-3 (ruas 20).
//   ruas 17 25 cm  jog ke TIMUR sampai tembok. BELOKNYA BLK_BALIK, bukan
//                  BLK_KIRI: ruas 16 berakhir menghadap BARAT, jadi perlu
//                  SETENGAH putaran untuk sampai ke TIMUR. Satu-satunya
//                  BLK_BALIK di tabel. Kalau pivot 180 der terbukti mahal di
//                  arena, yang diubah cara mengangkat K-3, bukan baris ini.
//                  HNT_DEPAN, dan ini yang PALING
//                  DICURIGAI dari ketiganya: hantu sensor depan yang tercatat
//                  6 Sep muncul saat robot menghadap BARAT, dan ruas 9 lama
//                  yang gagal berulang menghadap TIMUR. Kalau ruas ini
//                  berhenti seketika tanpa ada tembok, itu hantunya, bukan
//                  temboknya -- ganti ke HNT_ODO dengan abaikanDepan.
//   ruas 18        R-6, "maju hingga di tengah lantai pecah". Panjang PENUH
//                  R-6 pernah terukur 62 cm dari ujung ke ujung (BUKAN 55
//                  seperti guidebook; saksi keduanya sensor BELAKANG yang
//                  membaca 63 cm saat odometri menunjuk 62,2 -- dua alat yang
//                  tak berhubungan sepakat dalam 1 cm). Tapi ruas ini berhenti
//                  DI TENGAHNYA dan berangkat dari titik yang lain, jadi 62
//                  tidak bisa dipakai apa adanya. Sejak 8 Sep ruas ini
//                  berjalan sampai SZ-3, bukan "ke tengah", dan diisi 66.
//                  R-6 TERNYATA BERDINDING, bertentangan dengan catatan lama
//                  "tidak ada dinding yang bisa diikuti". Yang benar: dinding
//                  baru muncul sesudah ~11 cm pertama; di titik berangkat
//                  memang lima dari enam sensor gagal. Sesudah masuk, keenam
//                  sensor sah. Dinding KANAN duduk di 11-13 cm sepanjang ruas,
//                  yaitu DI DALAM pita wall.min, sehingga aturan "terlalu
//                  dekat" memicu putaran menjauh terus-menerus dan MENGALAHKAN
//                  kunci kompas: yaw berayun 348 -> 337 -> 0 -> 3, simpang
//                  sampai 15 der. KMD_TENGAH di baris ini WAJIB, bukan pilihan
//                  -- diuji dengan kemudi dinding KANAN dulu, dan itu justru
//                  yang menyeret robot 14 der ke kiri dalam dua lompatan.
//                  Zigzagnya tidak merusak jarak (cos 15 der = 3%) dan roll
//                  tetap -178 +/- 0,3 der: tidak ada ancaman terguling.
//                  Sensor DEPAN sekali membaca "dinding 34 cm" lalu kehilangan
//                  ia sama sekali sesudah robot berputar 23 der -- pantulan
//                  serong, bukan ujung ruas. Jangan pakai sensor depan di ruas
//                  ini; itu sebabnya ia HNT_ODO dan buta.
//   ruas 19        SZ-3 taruh korban K-3. DITAMBAHKAN 8 Sep 2026.
//   ruas 20,21,22  MENDEKATI K-4, ketiganya DITAMBAHKAN 8 Sep 2026: geser ke
//                  dinding kanan (20 cm), maju 20 cm, lalu angkat. Pola yang
//                  sama dengan pendekatan K-3 di ruas 14/15 -- ruas geser dan
//                  ruas maju dipisah karena HNT_SISI tidak bisa sekaligus
//                  maju.
//   ruas 23        JALAN KE DEPAN TANGGA, menggantikan ruas R-8 yang dibuang.
//                  Panjangnya -1: ukur dengan "m6 23".
//                  abaikanDepan WAJIB menyala di sini, dan ini pelajaran yang
//                  dibayar mahal di ruas R-8 yang lama: dengan sensor depan
//                  aktif, mode arena membaca TANGGA sebagai halangan lalu
//                  mencetak "halangan depan -> belok ke TIMUR" dan memutar
//                  robot 90 der di tengah lompatan. Terjadi sungguhan
//                  6 Sep 2026, persis seperti yang diperingatkan catatan
//                  abaikanDepan di bawah.
//                  DIBUANG 8 Sep 2026, dicatat supaya tidak dihidupkan lagi
//                  tanpa sebab: sepasang PIVOT PELURUS (BARAT lalu SELATAN,
//                  net nol) yang dulu duduk sebelum R-8. Manuvernya sendiri
//                  terbukti -- 'o3' lalu 'o2' menurunkan simpang dari 15 der
//                  jadi 2-4 der di R-6, 6 Sep 2026 -- jadi kalau simpang arah
//                  sesudah R-6 kembali jadi masalah, itu obatnya.
//   ruas 24 20 cm  RATAKAN ke dinding KANAN sebelum naik tangga. Satu-satunya
//                  angka HNT_SISI yang sudah punya nilai, dan ia datang
//                  langsung dari operator. 20 cm aman di atas wall.min (12),
//                  jadi penjaga arah geser tidak akan menolaknya tepat sebelum
//                  sasaran -- tabelSiap() memeriksa itu supaya kegagalannya
//                  muncul di meja, bukan di tengah arena.
//   ruas 25        R-9 TANGGA -- DICOBA 6 Sep 2026, GAGAL di 62 dari ~103 cm.
//                  Panjang bidangnya BUKAN 90: 90 cm itu proyeksi mendatar,
//                  dan odometri mengukur sepanjang badan yang menanjak, jadi
//                  yang terbaca ~90/cos(27 der) = ~103 cm.
//                  Pitch terukur naik bertahap 5,9 -> 13,0 -> 24,9 -> 27,3.
//                  TIGA cara dicoba, ketiganya punya cacatnya sendiri:
//                  (1) 'P' (kemudi dinding + kompas): roll memburuk ke -169,6
//                      (miring 10,4 der) dan yaw terseret -10 der TIAP
//                      lompatan. Kemudi samping TIDAK BERGUNA di tangga.
//                  (2) pivot 'o2' + 'w' manual: roll PULIH ke -179,1 (datar)
//                      -- cara terbaik sejauh ini. Tapi pivotnya melampaui
//                      sasaran ~9 der dua kali berturut-turut. SEBABNYA
//                      PENTING: tabel kompas dicatat di lantai DATAR, dan pada
//                      pitch 27 der sumbu yaw ikut miring, jadi "SELATAN
//                      351,7" tidak lagi menunjuk arah fisik yang sama.
//                      Kompas arena TIDAK SAH di atas bidang securam ini.
//                  (3) 'w' tanpa pivot dan tanpa kemudi: satu lompatan 12,6 cm
//                      membuat roll melompat ke -159,9 dan yaw berputar 30 der
//                      sekaligus. Robot harus DIANGKAT TANGAN keluar dari
//                      tangga -- tidak ada perintah mundur yang aman di sana.
//                  YANG BELUM DICOBA: kompensasi pose badan ('r0 <-pitch> 0')
//                  supaya badan tetap datar terhadap gravitasi sementara kaki
//                  bekerja di bidang miring. BODY_MAX_ROT_DEG = 20 der,
//                  tangganya 27 der, jadi kompensasinya tidak bisa penuh. Pose
//                  badan diterapkan SESUDAH gait dan bertahan selama berjalan,
//                  tapi 'b' dan '0' meresetnya.
//
// ====================================================================
// PERINGATAN SENSOR DEPAN -- berlaku untuk ruas 13, 17, 32, dan 33.
//
// Sensor depan MEMBERI ANGKA YANG BERBEDA TERGANTUNG ARAH HADAP, dan itu
// sudah terukur. Di SATU titik yang sama di ujung R-6, tanpa robot berpindah
// sedikit pun: menghadap SELATAN, 'j5' memberi 340 sampel 100% signal fail
// (bacaan yang BENAR untuk ruang kosong); menghadap BARAT, 'j5' memberi 340
// sampel 100% sah, 83-91 mm, sebaran 8 mm -- benda padat yang sangat mantap
// 8,6 cm di depan, di tempat yang operator pastikan kosong.
//
// Karena hantunya HILANG saat robot berputar, ia bukan kaca sensor dan bukan
// bagian robot -- keduanya ikut berputar. Apa yang dipantulkannya BELUM
// TERJAWAB, dan selama itu tiap ruas HNT_DEPAN adalah ruas yang bisa berhenti
// seketika di tempat yang salah tanpa gejala lain.
//
// Sensor sisi dan sensor depan juga pernah berselisih 27 cm di titik yang
// sama: sensor depan membaca 13 cm ke barat sementara kedua sensor kiri
// membaca 40 cm. Salah satunya buta terhadap sesuatu; belum diketahui mana.
//
// ====================================================================
// KAPASITAS CAPIT -- sebab K-4 dilewati, dan ini aritmetika, bukan selera.
//
// Robot punya DUA capit. Di sepanjang HOME..kaki tangga ada empat korban
// dengan hanya dua safe zone yang bisa dijangkau sebelum tangga: SZ-1 (di
// dalam R-4, dipakai untuk K-1) dan SZ-3 (di dalam R-8, dipakai untuk K-3).
// Jadi saat robot melewati K-4, capit depan masih memegang K-2 dan capit
// belakang memegang K-3. K-4 tidak punya tangan kosong yang menunggunya.
//
// K-3 memakai ARM_BELAKANG -- dan lengan itu HANYA punya grip (4 servo depan,
// 1 belakang, dikonfirmasi 7 Sep 2026). Sekuens untuk ARM_BELAKANG karena itu
// tidak boleh memanggil moveArmTarget(): ia menolak. Yang mengatur tinggi dan
// jangkauan capit belakang adalah letak BADAN, jadi ruas yang memakainya harus
// berhenti tepat di posisi angkat -- tidak ada sendi yang bisa mengoreksi. Tabel yang lebih lama menulis SEMUANYA ARM_DEPAN,
// yang membuat robot seolah punya satu capit saja -- tabelSiap() sekarang
// mensimulasikan isi kedua capit sepanjang tabel dan menolak yang begitu.
// ====================================================================
// AUDIT SELURUH TABEL, 6 Sep 2026 -- tiga baris lagi rusak dengan pola yang
// sama, ditemukan dengan membaca bab PENILAIAN, bukan bab ARENA:
//
//   "Untuk rintangan R4, R5, R8, R10, walaupun robot belum sepenuhnya keluar
//    rintangan namun berhasil menempatkan korban pada Safety-Zonenya..."
//
// Kalimat itu menyatakan KEEMPAT safe zone berada DI DALAM rintangannya:
// SZ-1 di R-4, SZ-2 di R-5, SZ-3 di R-8, SZ-4 di R-10. Tidak satu pun dari
// mereka ruas berjalan tersendiri.
//
// - SZ-4 (dulu HNT_DEPAN 40) punya cacat yang PERSIS sama dengan SZ-1 yang
//   menerbangkan robot keluar arena. Sekarang HNT_LANGSUNG di dalam R-10.
// - SZ-3 DIKEMBALIKAN sebagai tempat menaruh K-3, dari capit BELAKANG. Ia
//   dicabut lebih dulu karena mapping tidak melihatnya -- tapi ia memang tak
//   terlihat: petak 20x20 SEJAJAR LANTAI yang tertutup koral putih tidak
//   menghasilkan apa pun di LiDAR. Tanpa baris ini, K-3 diangkat lalu dibawa
//   sampai finish tanpa pernah ditaruh, dan poinnya hangus.
// - R-9 TANGGA 90 -> 103 cm. 90 itu proyeksi MENDATAR (guidebook: "M2
//   horizontal 90cm, vertical 50cm"), sedangkan odometri menghitung langkah
//   kaki SEPANJANG BIDANG MIRING: sqrt(90^2 + 50^2) = 103,0 cm. Dengan 90,
//   HNT_ODO berhenti 13 cm sebelum puncak -- yaitu DI ATAS TANGGA, tempat
//   paling buruk untuk berhenti. Ini geometri dari dua angka guidebook, bukan
//   pengukuran; percobaan 6 Sep berhenti di 62 cm jadi puncaknya belum pernah
//   disentuh.
//
// MASIH TERSISA, sengaja tidak diubah karena butuh mata di arena:
// - ruas 33 (SZ-5/FINISH) masih HNT_DEPAN 40, dan ia yang PALING JAUH dari
//   pemeriksaan: SZ-5 ada di BIDANG MIRING dengan dinding 10 cm di sisi dan
//   belakang, jadi berhenti pada dinding depan masuk akal -- tapi angka 40-nya
//   belum pernah diperiksa, dan berkas sensor depan di bidang miring menembak
//   lantai.
// - BENTROK LAMA YANG IKUT HILANG: ruas "13 cm dari R-5 ke R-6" bertentangan
//   dengan guidebook ("Jarak antara sisi luar tanggul R6 dan sisi luar tanggul
//   R5 sejauh 52cm") dan selisih 13 lawan 52 tidak pernah terjelaskan.
//   Lintasan 6 Sep (malam) tidak lagi melewati potongan itu, jadi bentroknya
//   gugur dengan sendirinya -- tapi kalau suatu saat jalur lama dipakai lagi,
//   pertanyaannya belum terjawab.
// ====================================================================
// DIBACA ULANG DARI GUIDEBOOK 6 Sep 2026 -- dan ia membantah tabel ini di
// empat tempat. Sumber: "Guidebook SAR.pdf", bab ARENA, JALUR MISI, PENILAIAN.
//
// 1. R-7 BUKAN LORONG. Ia tugas MEMBERSIHKAN batu koral dari SZ-3:
//    "Pembersihan ini sebagai Rintangan 7 (R7)". Bab PENILAIAN menegaskannya
//    dari sisi lain: "Melewati rintangan (R1-R11 SELAIN R7, R9)" -- R-7 tidak
//    masuk hitungan "dilewati" karena memang bukan sesuatu yang dilewati.
//    Ruas yang menuju tangga karena itu bernama R-8, bukan R-7.
// 2. SZ-1 ADA DI DALAM RUANG R-4: "Ruang R4: 45 x 60cm. SZ area 20 x 20cm."
//    Ia bukan ruas berjalan sesudah R-4. Ini yang membuat robot keluar arena
//    6 Sep 2026 -- barisnya menyuruh berjalan mencari dinding 40 cm di depan.
// 3. SZ-3 ADA DI DALAM R-8: "Ruang R8 sebesar 73x51cm namun tidak meliputi
//    SZ-3", tingginya SEJAJAR LANTAI dan tertutup koral putih. Karena itu
//    mapping 6 Sep tidak melihatnya: petak 20x20 setinggi nol tidak
//    menghasilkan apa pun di LiDAR. SZ-3 ADA, hanya tak terlihat sensor.
// 4. K-3 DAN K-4 DI SAMPING R-6: "R6 bersebelahan dengan K-3 dan K-4",
//    tertimpa dua papan 14x17x2 cm, titik tengah korban 9 cm dari sisi R-6.
//    Mereka bukan titik di sepanjang lintasan sesudah R-6.
//
// YANG PALING BERBAHAYA, dan belum ditangani sama sekali:
//    "Arah robot di Home sesuai permintaan juri." -- ARAH BERANGKAT DIPILIH
//    JURI, sedangkan MISI_ARAH_BERANGKAT di Misi.h dipaku ke 0 (UTARA).
//    Kalau juri meminta arah lain, SELURUH kolom arah mutlak bergeser dan
//    robot berjalan ke arah yang salah sejak ruas pertama.
//
// Ukuran arena: alas 3,6 x 2,4 m, lebar lorong 45 cm, tebal dinding 2 cm,
// tinggi dinding 10 cm. Dipakai sebagai pagar akal sehat: tidak ada ruas yang
// masuk akal lebih panjang dari itu (lihat MISI_RUAS_MAKS_CM).
//
// R-11: "Lebar jalan pada R11 yang bisa dilalui kaki robot sebesar 50-10-10
// atau 30cm" -- 30 itu LEBAR, bukan panjang. Panjang ruas 31 tetap belum ada.
//
// Skor yang perlu diingat saat memutuskan risiko: R-9 (tangga) bernilai 150
// tanpa korban dan 300 dengan korban -- rintangan termahal di seluruh arena.
// Membersihkan SZ-3 (R-7) 100 bila korban K-3 ditempatkan di sana, 200 bila
// seluruh areanya bersih.
// ====================================================================
// Sisanya (panjang lorong, jarak antar-ruang) tidak ada di guidebook.
//
// PROFIL KAIL, ruas 24 (R-9), dipasang 9 Sep 2026. Satu-satunya profil yang
// bentuknya TIDAK seragam: dua kaki depan naik ke tapak di atas sambil
// membuka ke depan untuk mengait, dua kaki belakang MEMANJANG KE BAWAH
// karena masih di tapak yang lebih rendah, kaki tengah menanggung badan.
// Selisih depan-belakang itulah yang membuat badan tetap DATAR di tanjakan
// (25,7 der terkompensasi dari bidang 27,3 der). Angkanya KAIL_* di config.h, dan yang sudah dibuktikan baru bahwa IK
// sanggup mencapainya sepanjang satu siklus gait penuh (cek_kail.cpp) --
// BELUM ada satu pun percobaan di tangga.
//
// PANJANG 103 CM SEKARANG MERAGUKAN, dan ini konsekuensi aturan di bawah:
// mengganti profil membatalkan panjang ruasnya. 103 datang dari geometri
// (sqrt(90^2+50^2)), bukan dari odometri, tapi ruas ini BUTA dan HNT_ODO --
// yang menghentikannya cuma odometri, dan odometri di bidang 27 der dengan
// panjang langkah yang berbeda tidak membaca sama. Ukur ulang dengan "m6 24"
// begitu profil ini pernah benar-benar menaiki tangganya.
//
// PROFIL: TANGGA dipakai untuk semua permukaan yang butuh kaki diangkat lebih
// tinggi -- jalan pecah, berpuing, berlumpur, dan anak tangga. MERUNDUK untuk
// bidang miring: badan turun 20 mm menurunkan titik berat DAN melipat kaki,
// sehingga sisa jangkauan ke bawah bertambah di bibir turunan.
//
// abaikanDepan: SENGAJA hanya di ruas yang dibatasi ODOMETRI. Di ruas kasar,
// halangan depan membuat mode arena PINDAH MATA ANGIN dan sisa jaraknya akan
// diukur ke arah yang salah; di turunan, berkas sensor depan menembak lantai.
// Robot berjalan buta ke depan sepanjang ruas itu, jadi harus ada yang lain
// yang menghentikannya -- tabelSiap() menolak tabel yang melanggar ini.
// ====================================================================
// >>> TABEL LINTASAN BAKU: acuan di FLASH, bukan yang dijalankan.
// Yang dijalankan misi adalah salinannya di RAM (RUAS[] di bawah), yang boleh
// diubah dari HUD lewat 'm5s'. 'm5r' memulangkan RAM ke tabel ini.
const Ruas RUAS_BAKU[] = {
//         nama                             belok      kemudi      profil        buta   henti          nilai            aksi           lengan    pivot
/*0*/    { "HOME -> samping K-1",           BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_BELAKANG,  32,              AKS_TIDAK_ADA, 0 },
/*1*/    { "mundur jika bisa",              BLK_KIRI,  KMD_TENGAH, PRF_DATAR,    false, HNT_MUNDUR,     9,              AKS_TIDAK_ADA, 0 },
/*2*/    { "K-1 angkat korban",             BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_LANGSUNG,   0,              AKS_AMBIL,     ARM_DEPAN,  0.0f, false, false, 35.0f, 30.0f },
/*3*/    { "R-1 jalan pecah",               BLK_KANAN, KMD_KIRI,   PRF_TANGGA,   true,  HNT_ODO,       86,              AKS_TIDAK_ADA, 0 },
/*4*/    { "M1 turunan + R-2/R-3",          BLK_LURUS, KMD_KANAN,  PRF_DATAR,    true,  HNT_ODO,       58,              AKS_TIDAK_ADA, 0 },
/*5*/    { "Depan sampai 30 cm",            BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_DEPAN,     30,              AKS_TIDAK_ADA, 0 },
/*6*/    { "SZ-1 taruh korban (dalam R-4)", BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_LANGSUNG,   0,              AKS_TARUH,     ARM_DEPAN, -20.0f },
/*7*/    { "pivot kiri ratakan dinding",    BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_DEPAN,     22,              AKS_TIDAK_ADA, 0,         +20.0f },
/*8*/    { "hadap kiri maju",               BLK_KIRI,  KMD_KANAN,  PRF_TANGGA,   false, HNT_ODO,       30,              AKS_TIDAK_ADA, 0 },
/*9*/    { "mundur jika bisa",              BLK_KIRI,  KMD_TENGAH, PRF_TANGGA,   false, HNT_MUNDUR,     9,              AKS_TIDAK_ADA, 0 },
/*10*/   { "K-2 angkat korban",             BLK_LURUS, KMD_TENGAH, PRF_TANGGA,   false, HNT_LANGSUNG,   0,              AKS_AMBIL,     ARM_DEPAN,   0.0f, false, false, 35.0f, 30.0f }, 
/*11*/   { "R-5 lumpur: BARAT sampai ujung",BLK_KANAN, KMD_KANAN,  PRF_TANGGA,   false, HNT_ODO,       22,              AKS_TIDAK_ADA, 0 },
/*12*/   { "SZ-2 taruh korban (kanan 20)",  BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_LANGSUNG,   0,              AKS_TARUH,     ARM_DEPAN, -10.0f },
/*13*/   { "SELATAN keluar R-5 (kiri 20)",  BLK_KIRI,  KMD_KANAN,  PRF_TANGGA,   false, HNT_ODO,       30,              AKS_TIDAK_ADA, 0,         +10.0f },
/*14*/   { "ratakan ke dinding KANAN",      BLK_LURUS, KMD_KANAN,  PRF_MERUNDUK, false, HNT_SISI,      13,              AKS_TIDAK_ADA, 0,           0.0f, false, false },
/*15*/   { "SELATAN sampai tembok K-3",     BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_ODO,       28,              AKS_TIDAK_ADA, 0 },
/*16*/   { "putar kiri lalu maju",          BLK_KIRI,  KMD_TENGAH, PRF_DATAR,    false, HNT_DEPAN,     13,              AKS_TIDAK_ADA, 0 },
// /*14*/   { "kiri ke depan K-3",             BLK_BALIK, KMD_KANAN,  PRF_TANGGA,   false, HNT_SISI,      45,              AKS_TIDAK_ADA, 0 },
// /*15*/   { "maju sedikit (CARI yang halus)",BLK_LURUS, KMD_TENGAH, PRF_TANGGA,   false, HNT_ODO,        8,              AKS_TIDAK_ADA, 0 },
// /*16*/   { "K-3 angkat korban",             BLK_LURUS, KMD_TENGAH, PRF_TANGGA,   false, HNT_LANGSUNG,   0,              AKS_AMBIL,     ARM_DEPAN,   0.0f, true,  true,  35.0f, 20.0f },
// /*17*/   { "hadap SELATAN (rotasi saja)",   BLK_KIRI,  KMD_KIRI,   PRF_TANGGA,   false, HNT_LANGSUNG,   0,              AKS_TIDAK_ADA, 0 },
// /*18*/   { "R-6 pecah: SELATAN sampai 28",  BLK_LURUS, KMD_KIRI,   PRF_TANGGA,   false, HNT_DEPAN,     28,              AKS_TIDAK_ADA, 0 },
// /*19*/   { "SZ-3 taruh korban",             BLK_LURUS, KMD_KIRI,   PRF_DATAR,    false, HNT_LANGSUNG,   0,              AKS_TARUH,     ARM_DEPAN },
// /*20*/   { "hadap BARAT (rotasi saja)",     BLK_KANAN, KMD_KANAN,  PRF_TANGGA,   false, HNT_LANGSUNG,   0,              AKS_TIDAK_ADA, 0 },
// /*21*/   { "K-4: pendekatan milik CARI",    BLK_LURUS, KMD_KIRI,   PRF_TANGGA,   false, HNT_LANGSUNG,   0,              AKS_TIDAK_ADA, 0 },
// /*22*/   { "K-4 angkat korban",             BLK_LURUS, KMD_KIRI,   PRF_TANGGA,   false, HNT_LANGSUNG,   0,              AKS_AMBIL,     ARM_DEPAN,   0.0f, true,  true,  35.0f, 20.0f },
/*17*/   { "jalan sampai bebatuan",         BLK_KANAN, KMD_KIRI,   PRF_TANGGA,   true,  HNT_ODO,       76,              AKS_TIDAK_ADA, 0 },
/*18*/   { "kanan maju",                    BLK_KANAN, KMD_KIRI,   PRF_DATAR,    true,  HNT_ODO,       25,              AKS_TIDAK_ADA, 0 },
/*19*/   { "ratakan 13 cm dinding KANAN",   BLK_KIRI,  KMD_KANAN,  PRF_DATAR,    false, HNT_SISI,      13,              AKS_TIDAK_ADA, 0 },
/*20*/   { "R-9 TANGGA (miring 103)",       BLK_LURUS, KMD_KANAN,  PRF_TANJAK,   true,  HNT_PUNCAK,     0,              AKS_TIDAK_ADA, 0 },
/*21*/   { "Maju sedikit naik tangga",      BLK_LURUS, KMD_KANAN,  PRF_TANJAK,   true,  HNT_ODO,       10,              AKS_TIDAK_ADA, 0 },
/*22*/   { "R-10 puing+lumpur miring",      BLK_LURUS, KMD_KANAN,  PRF_TANGGA,   true,  HNT_ODO,       22,              AKS_TIDAK_ADA, 0 },
// /*21*/   { "jalan ke kiri ke depan SZ-4",   BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_SISI,      18,              AKS_TIDAK_ADA, 0 },
// /*22*/   { "SZ-4 taruh korban (dalam R-10)",BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_LANGSUNG,   0,              AKS_TARUH,     ARM_DEPAN, +20.0f },
/*23*/   { "jalan ke kanan depan K-5",      BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_SISI,      13,              AKS_TIDAK_ADA, 0 },
/*24*/   { "K-5 angkat korban",             BLK_LURUS, KMD_KANAN,  PRF_DATAR,    false, HNT_LANGSUNG,   0,              AKS_AMBIL,     ARM_DEPAN, 0.0f, false, false, 30.0f, 30.0f },
/*25*/   { "maju sedikit",                  BLK_KIRI,  KMD_TENGAH, PRF_TANGGA,   false, HNT_BELAKANG,  18,              AKS_TIDAK_ADA, 0 },
/*25*/   { "R-11 longsor (lebar 30)1/2",    BLK_LURUS, KMD_TENGAH, PRF_SEMPIT,   false, HNT_ODO,       45,              AKS_TIDAK_ADA, 0 },
/*26*/   { "R-11 longsor (lebar 30)2/2",    BLK_LURUS, KMD_TENGAH, PRF_SEMPIT,   false, HNT_DEPAN,     20,              AKS_TIDAK_ADA, 0 },
/*27*/   { "SZ-5 / FINISH",                 BLK_KANAN, KMD_KIRI,   PRF_DATAR,    false, HNT_DEPAN,     30,              AKS_TARUH,     ARM_DEPAN },
};

// <<< TABEL LINTASAN BAKU selesai

// --- TABEL YANG BENAR-BENAR DIJALANKAN, di RAM ----------------------------
//
// Salinan RUAS_BAKU[] yang boleh diubah dari HUD lewat 'm5s', jadi menyetel
// lintasan tidak lagi menuntut satu putaran compile + flash per percobaan.
// Isinya diisi tabelBaku() saat konstruktor Misi jalan; sebelum itu nol.
//
// Nama tidak boleh tetap menunjuk ke flash: begitu operator menggantinya, ia
// harus menunjuk ke RAM. Kolam nama ini yang dipakai SEJAK AWAL -- termasuk
// untuk nama yang belum diubah -- supaya cuma ada satu jenis pointer di tabel
// dan tidak ada yang perlu mengingat mana yang boleh ditulisi.
Ruas RUAS[RUAS_MAKS];
static char NAMA_RAM[RUAS_MAKS][RUAS_NAMA_MAKS];

// JUMLAHNYA BOLEH BERUBAH lewat 'm5+' / 'm5-'. Peta poin di Skor.cpp ikut
// digeser di fungsi yang sama -- lihat skorSisip()/skorHapus().
uint8_t RUAS_N = 0;                  // diisi tabelBaku()
const uint8_t RUAS_BAKU_N = sizeof(RUAS_BAKU) / sizeof(RUAS_BAKU[0]);
static_assert(sizeof(RUAS_BAKU) / sizeof(RUAS_BAKU[0]) <= RUAS_MAKS,
              "Tabel lintasan lebih panjang dari _cm[] -- naikkan RUAS_MAKS di Misi.h");

// Plafon kedua, dan ia BUKAN soal RAM. RUAS_N, _i dan _iAkhir semuanya
// uint8_t, dan _iAkhir memakai 255 sebagai penanda "sampai ruas terakhir".
// Pada 256 baris, RUAS_N di atas terpotong jadi 0 DIAM-DIAM: misi langsung
// selesai tanpa satu ruas pun dijalankan, dan penjaga di atas tidak
// menangkapnya karena ia membandingkan sizeof, bukan RUAS_N yang terpotong.
//
// Menaikkan RUAS_MAKS sendiri murah -- 9 byte per slot (_arah 1 + _serong 4
// + _cm 4), 36 slot cuma 324 byte, dan EEPROM tidak ikut tersentuh. Batas
// inilah yang tidak bisa dibeli dengan RAM.
static_assert(sizeof(RUAS_BAKU) / sizeof(RUAS_BAKU[0]) <= 254,
              "Tabel > 254 baris: RUAS_N/_i/_iAkhir uint8_t, dan 255 dipakai "
              "_iAkhir sebagai penanda. Lebarkan ketiganya ke uint16_t dulu.");

// ====================================================================
// AMBANG
// ====================================================================

// Berapa sampel LiDAR BERTURUT-TURUT sebelum pemicu jarak dipercaya.
// Kecil dengan sengaja: LidarArray sudah menyaring hantu lewat median-3 dan
// LIDAR_MIN_CM, jadi ini cuma lapis terakhir terhadap satu pantulan nyasar.
static const uint8_t MISI_SAMPEL_N = 3;

// JEDA SESUDAH GANTI PROFIL, sebelum LiDAR ruas itu dipercaya. Mengganti
// profil menggerakkan BADAN: standHeight beda 35 mm antara DATAR dan TANGGA,
// standRadius 25 mm ke SEMPIT. Selama badan turun/naik, seluruh berkas sensor
// ikut berayun -- yang depan menyapu naik-turun di dinding, yang samping
// menjauh/mendekat tanpa robot berpindah sesenti pun. Pemicu yang dibaca di
// tengah ayunan itu memberhentikan ruas di tempat yang salah.
//
// DUA tunggu yang berurutan, bukan satu:
//   1. ramp profil selesai -- ditanyakan ke gait (profilTenang()), bukan
//      ditebak dari jam, supaya ia ikut kalau 'gait.profile_tau' disetel.
//   2. sesudah itu BARU histori median LiDAR diisi ulang. Nilainya 3 sampel
//      penuh: getDistance() menahan diri sampai _histN >= 3, dan tiap kanal
//      kebagian giliran tiap NUM_LIDAR x LIDAR_PERIOD_MS.
static const uint32_t MISI_LIDAR_SEGAR_MS = 3UL * NUM_LIDAR * LIDAR_PERIOD_MS;

// Pagar kalau ramp profil tidak kunjung selesai -- gait tidak di-update,
// servo lemas, atau profil target diubah dari luar di tengah tunggu.
static const uint32_t MISI_SETEL_BATAS_MS = 5000;

// Batas waktu SATU ruas. Longgar karena ruas kasar dilalui dengan profil
// lambat: TANGGA menaikkan cycleTime +400 ms, MERUNDUK +200 ms.
static const uint32_t MISI_RUAS_BATAS_MS = 90000;

// PAGAR JARAK satu ruas. Batas waktu saja TIDAK CUKUP: pada ~10 cm/detik,
// 90 detik berarti 9 METER, sedangkan seluruh arena cuma 3,6 x 2,4 m
// (guidebook, bab ARENA). Jadi ruas yang syarat hentinya tidak pernah
// terpenuhi bukan berhenti -- ia berjalan keluar arena.
//
// Itu bukan dugaan. 6 September 2026, ruas SZ-1 berhenti pada HNT_DEPAN 40 cm;
// robot terlanjur berada di tempat yang tak ada dindingnya, sensor depan tidak
// pernah membaca 40, dan robot berjalan terus keluar arena sampai operator
// mematikannya.
//
// 200 cm dipilih karena ruas terpanjang yang pernah terukur 120 cm, jadi pagar
// ini tidak akan pernah menyala pada ruas yang sehat, tapi menghentikan yang
// kabur dalam dua meter -- masih di dalam arena.
static const float MISI_RUAS_MAKS_CM = 200.0f;

// Batas waktu SELURUH misi. Kontes memberi 5 menit (300 detik) dan waktu itu
// juga yang dipakai menghitung bonus: skor x 300 / waktu. Lewat dari itu,
// berjalan terus tidak menambah apa pun -- lebih baik berhenti dengan sebab
// yang tercetak daripada mati di tengah arena.
static const uint32_t MISI_TOTAL_BATAS_MS = 300000;

// SEBERAPA SERONG masih boleh selama ruas berjarak. Jauh lebih longgar dari
// HEADING_TOLERANCE_DEG (6 der): angka itu milik pemeriksaan "pivot sudah
// sampai", tempat robot berdiri DIAM. Di sini robot berjalan di atas koral
// dan kelereng, tempat kaki tergelincir tiap langkah.
//
// Ambangnya turunan dari kerugian odometri, bukan selera: simpang theta
// membuat odometer mengukur sepanjang badan sementara ruas diukur sepanjang
// lorong, jadi jarak nyata = terbaca x cos(theta). Pada 25 der itu 9%.
static const float MISI_SERONG_DEG = 25.0f;

// Berapa lama boleh seserong itu. Diukur dalam SIKLUS GAIT, bukan milidetik
// tetap: koreksi heading hanya bisa bekerja selewat langkah, dan profil
// TANGGA memakai cycleTime dua kali lipat DATAR.
static const uint8_t  MISI_SERONG_SIKLUS = 3;
static const uint32_t MISI_SERONG_MIN_MS = 3000;

// ====================================================================
// SEKUENS LENGAN -- TAHAP 1: GERBANG JARAK + POSE TETAP
//
// Robot sudah BERHENTI di sini, dan kamera sudah meluruskan kiri-kanan
// sebelum ruas ini. Yang tersisa buat lengan cuma memainkan pose
// yang sudah disetel. TIDAK ADA IK dari sensor, dan tidak ada umpan balik
// posisi dari servo mana pun -- karena itu tiap langkah diberi waktu tetap.
//
// Pose dinyatakan sebagai SUDUT SENDI, hard-coded di config.h
// (KORBAN_SIAP_*, KORBAN_JEPIT_*, KORBAN_LEPAS_*). Bukan IK: yang dibidik
// dengan tangan di robot adalah sudutnya, dan sudut itu tidak boleh ikut
// bergeser saat KORBAN_CAPIT_MM atau tinggi profil disetel. Dulu pose ini
// titik capit lewat moveArmGrip(), dan tinggi badan profil ikut menentukan --
// benar untuk satu profil, meleset 15 mm untuk yang lain, DIAM-DIAM.
//
// Harganya: jangkauan capit tidak lagi mengikuti jarak berhenti. Memindahkan
// tempat robot berhenti sekarang MENUNTUT sudut sendi disetel ulang dengan
// tangan -- tidak ada lagi yang menghitungnya sendiri.
//
// DAN SEJAK 18 Sep 2026 TEMPAT ITU TIDAK LAGI DI TABEL. Seluruh baris AMBIL
// memakai HNT_LANGSUNG: ruas AMBIL tidak berjalan sama sekali, dan yang
// menaruh robot pada jarak yang benar adalah ruas-ruas SEBELUMNYA. Jadi kalau
// capit meleset, yang disetel panjang ruas sebelumnya -- bukan baris AMBIL,
// yang tidak lagi punya angka jarak untuk disetel.
//
// KORBAN_JARAK_CM karena itu tidak lagi dipakai satu baris tabel pun. Ia masih
// menurunkan KORBAN_CAPIT_MM, yang hanya dibaca cek_korban.cpp -- pemeriksa
// amplop di PC, bukan firmware.
//
// setSudutLengan() menandai sudut yang keluar 0..180 servo, tapi TETAP
// mengirimkannya; amplop IK-nya sendiri masih disapu cek_korban.cpp.
// ====================================================================

// Fase dari waktu. Sekuens ini buta, jadi "sudah sampai?" tidak bisa ditanya
// ke siapa pun -- yang bisa dilakukan cuma memberi tiap langkah jatahnya.
// langkah = jumlah fase yang SUDAH dikerjakan, jadi fase 0 jalan seketika dan
// fase yang terlewat (loop tersendat) tetap dikerjakan, tidak dilompati.
static bool faseBaru(uint8_t& langkah, uint32_t sejak, uint8_t& fase) {
    fase = (uint8_t)((millis() - sejak) / LENGAN_JEDA_MS);
    if (fase < langkah) return false;
    langkah = fase + 1;
    return true;
}

// Satu pose sendi, dengan sebabnya kalau mentok. Sudutnya TETAP dikirim --
// sama seperti perintah 'as' -- karena sekuens yang berhenti di tengah jalan
// meninggalkan capit menggantung di atas korban; yang mentok cuma ditandai.
//
// Penandaan ini wajib ada di sini: HexaArm::angleToPulse() meng-clamp DIAM-
// DIAM, tanpa penanda seperti _servoClamped milik kaki, jadi pose yang tidak
// sampai terlihat persis sama dengan pose yang sampai.
static bool poseSendi(Hexapod& robot, uint8_t lengan,
                      float bahu, float siku, float prg, const char* nama) {
    float sv[3];
    if (robot.setSudutLengan(lengan, bahu, siku, prg, sv)) return true;
    Serial.print("!! Pose lengan '"); Serial.print(nama);
    Serial.printf("' MENTOK: servo bahu %.1f, siku %.1f, pergelangan %.1f der",
                  (double)sv[0], (double)sv[1], (double)sv[2]);
    Serial.println(" -- yang di luar 0..180 di-clamp. Setel KORBAN_*_BAHU/SIKU/PRG.");
    return false;
}

// return true bila sekuens ini sudah selesai.
// `condong` = ruas ini menggeser BADAN maju dan meluruskannya sebelum lengan
// turun. `koreksiYaw` = berapa derajat badan harus diputar supaya menghadap
// heading ruas ini; dihitung pemanggil, karena di sinilah satu-satunya tempat
// yang TIDAK punya akses ke Navigation.
//
// TRANSLASI MAJU BERLAKU UNTUK SEMUA AMBIL sejak 16 Sep 2026, termasuk uji
// 'aa': keluhan "robot kurang maju saat capit turun" sama di tiap korban,
// bukan cuma di K-3/K-4 yang tertutup reruntuhan. `condong` sekarang cuma
// menyalakan DUA tambahan sesudahnya -- jeda konfirmasi mata dan pelurusan
// yaw -- karena keduanya memang hanya perlu di ruas yang korbannya tertutup.
//
// Harganya satu fase, yaitu satu LENGAN_JEDA_MS (2100 ms), per ruas AMBIL.
static bool sekuensAmbil(Hexapod& robot, uint8_t lengan, uint8_t& langkah,
                         uint32_t sejak, bool condong, float koreksiYaw,
                         float mundurMm, float condongMm) {
    // Kolom ruas menang atas param global. 0 di kolom berarti "belum diisi",
    // bukan "maju nol" -- lihat Ruas::condongMm.
    const float majuMm = (condongMm > 0.0f) ? condongMm : KORBAN_CONDONG_MM;
    uint8_t fase;
    if (!faseBaru(langkah, sejak, fase)) return false;

    // FASE 2 SELALU MENGGESER BADAN MAJU, dan sisanya bergeser nomornya.
    // Dinyatakan sebagai pergeseran nomor, bukan dua salinan switch: dua
    // salinan berarti dua tempat untuk lupa saat posenya disetel.
    //
    // DITUKAR DENGAN POSE JEPIT pada 16 Sep 2026, diminta R2C. Dulu badan maju
    // lebih dulu lalu lengan turun; sekarang lengan turun ke ketinggian jepit
    // DULU, baru badan mendorongnya masuk ke korban. Bedanya jalur yang
    // ditempuh capit: turun di tempat lalu maju mendatar, bukan turun sambil
    // sudah berada di atas korban.
    //
    //   fase 0  pose SIAP
    //   fase 1  pose JEPIT
    //   fase 2  BADAN MAJU        <- dulu di sini pose JEPIT
    //   fase 3  capit MENUTUP
    //   fase 4  ANGKAT + badan pulang  <- ditambah 17 Sep 2026
    //   fase 5  lipat ke REHAT
    if (fase >= 2) {
        if (fase == 2) {
            // BADAN MAJU, KAKI DIAM. Lihat condong.mm -- memajukan kaki
            // membuat lengan menabrak reruntuhan dalam perjalanan turunnya.
            //
            // SUMBU X DAN Z DIPERTAHANKAN, dan ini bukan kerapian: Raspi
            // menengahkan korban dengan menggeser BADAN menyamping
            // ('t<x> 0 0'), jadi X memegang seluruh koreksi vision yang baru
            // saja dibayar. Menulis setBodyTranslation(0, y, 0) menolkannya --
            // dan dari luar itu terlihat persis seperti "Raspi kehilangan
            // kendali dan badan kembali ke default sebelum mencapit",
            // laporan R2C 15 Sep 2026.
            //
            // PELAN, dan itu bukan kehalusan: badan yang menggeser cepat
            // sambil kaki diam menggoyang seluruh robot, tepat di atas korban
            // yang akan dicapit. Lajunya dikembalikan di fase terakhir.
            robot.setBodySlewMm(KORBAN_CONDONG_LAJU_MM_S);
            const Vec3 t0 = robot.bodyTransTarget();
            robot.setBodyTranslation(t0.x, majuMm, t0.z);
            Serial.printf("  badan condong %.0f mm ke depan pada %.0f mm/detik "
                          "(X %.0f mm milik Raspi DIPERTAHANKAN).\n",
                          (double)majuMm,
                          (double)KORBAN_CONDONG_LAJU_MM_S, (double)t0.x);
            return false;
        }

        // SISANYA HANYA UNTUK RUAS `condong`: jeda konfirmasi mata lalu
        // pelurusan yaw. Ruas AMBIL biasa cuma membayar satu fase, yaitu
        // translasi di atas, lalu langsung lanjut ke pose lengan.
        uint8_t nGeser = 1;
        if (condong) {
            // TIGA FASE, dan yang di tengah sengaja KOSONG.
            //
            // Diminta R2C 15 Sep 2026: beri jeda 2-3 detik antara translasi
            // maju dan rotasi badan, untuk konfirmasi mata sebelum lengan
            // turun. Jeda itu dinyatakan sebagai fase kosong dengan jatahnya
            // sendiri, bukan sebagai delay: sekuens ini tidak boleh memblokir
            // -- 'm0' dan 's' harus tetap bisa menghentikannya di detik mana
            // pun.
            //
            // Lebarnya terkunci ke LENGAN_JEDA_MS karena faseBaru() memakai
            // satu jatah untuk semua fase, jadi condong.jeda dibulatkan NAIK
            // ke jatah terdekat. Dibulatkan naik, bukan turun: jeda yang
            // diminta operator adalah jeda MINIMUM untuk memeriksa dengan mata.
            const uint8_t nJeda = (uint8_t)((KORBAN_CONDONG_JEDA_MS + LENGAN_JEDA_MS - 1)
                                            / LENGAN_JEDA_MS);
            // PELURUSAN YAW MEMBAYAR FASENYA SENDIRI, dan hanya kalau ia
            // benar-benar dikerjakan. Dulu fase ini tetap dipakai saat
            // condong.yaw 0: ia mencetak "DILEWATI" lalu return false, yang
            // artinya 2100 ms berlalu tanpa satu servo pun bergerak.
            //
            // Itu baru terasa sejak 17 Sep 2026, waktu Raspi mulai mengirim
            // 'Qcondong.jeda 0' dan 'Qcondong.yaw 0' tiap kali menyambung.
            // Dengan keduanya nol, ruas condong seharusnya tidak berbeda dari
            // ruas AMBIL biasa -- tapi ia masih membayar satu fase kosong.
            const uint8_t nYaw = KORBAN_CONDONG_YAW ? 1 : 0;
            if (fase >= 3 && fase < (uint8_t)(3 + nJeda)) return false;
            if (nYaw && fase == (uint8_t)(3 + nJeda)) {
                // DILURUSKAN. Lantai pecah membuat gait tidak pernah
                // benar-benar berhenti di mata angin ruasnya; sisa simpangan
                // itu yang dihabiskan di sini, di atas kaki yang diam.
                // Nilainya di-clamp setBodyRotation() ke BODY_MAX_ROT_DEG,
                // jadi simpangan yang terlalu besar diperbaiki SEBAGIAN --
                // bukan ditolak diam-diam.
                // ROLL DAN PITCH DIPERTAHANKAN. Stabilisasi IMU menulis
                // keduanya, dan menolkannya di sini menjatuhkan badan kembali
                // ke datar di atas lantai yang miring.
                const Vec3 r0 = robot.bodyRotTargetDeg();
                robot.setBodyRotation(r0.x, r0.y, koreksiYaw);
                Serial.printf("  badan diluruskan %+.1f der ke heading ruas.\n",
                              (double)koreksiYaw);
                return false;
            }
            // Satu fase dasar (translasi maju) + jeda + pelurusan. Dengan
            // condong.jeda 0 DAN condong.yaw 0 ini jatuh ke 1, yaitu persis
            // ruas AMBIL biasa -- kolom `condong` jadi gratis, bukan cuma
            // tidak berguna.
            nGeser = (uint8_t)(1 + nJeda + nYaw);
        }
        fase -= nGeser;
    }

    switch (fase) {
        case 0:   // servo lengan hidup, capit BUKA, mendekat DARI ATAS korban
            robot.armEnable(lengan, true);
            // 'm0' di tengah lipatan meninggalkan laju separuh terpasang.
            robot.setSlewLengan(lengan, ARM_SLEW_DEG_S);
            robot.setGrip(lengan, KORBAN_GRIP_BUKA);
            poseSendi(robot, lengan, KORBAN_SIAP_BAHU, KORBAN_SIAP_SIKU,
                      KORBAN_SIAP_PRG, "siap");
            // BADAN MUNDUR, MENUMPANG DI FASE INI. Bukan fase sendiri:
            // lengan dan badan itu aktuator yang berbeda dan bisa bergerak
            // bersamaan, jadi menumpang di sini memberi jalur turun yang
            // bebas dengan biaya nol detik. Fase sendiri akan menambah satu
            // LENGAN_JEDA_MS penuh ke SETIAP pengambilan.
            //
            // X DIPERTAHANKAN, alasannya sama dengan di fase BADAN MAJU:
            // X memegang koreksi vision dari Raspi, dan menolkannya di sini
            // membuang penengahan yang baru saja dibayar.
            if (mundurMm > 0.0f) {
                robot.setBodySlewMm(KORBAN_CONDONG_LAJU_MM_S);
                const Vec3 t0 = robot.bodyTransTarget();
                robot.setBodyTranslation(t0.x, -mundurMm, t0.z);
                Serial.printf("  badan MUNDUR %.0f mm dulu supaya busur turun "
                              "siku tidak memukul korban.\n", (double)mundurMm);
            }
            break;
        case 1:   // TURUN mengelilingi korban -- bukan menjulur mendatar, yang
            //    mendorong boneka menjauh sebelum capit sempat menutup
            poseSendi(robot, lengan, KORBAN_JEPIT_BAHU, KORBAN_JEPIT_SIKU,
                      KORBAN_JEPIT_PRG, "jepit");
            break;
        case 2:
            robot.setGrip(lengan, KORBAN_GRIP_TUTUP);
            break;
        case 3:   // ANGKAT keluar dari kantong, DAN badan pulang. Diminta
            //    R2C 17 Sep 2026.
            //
            // JEPIT ada di dalam kantong reruntuhan, dan lengan turun ke sana
            // lewat jalur yang sudah terbukti bersih (fase 0 ke fase 1).
            // Melipat LANGSUNG dari JEPIT ke REHAT menempuh jalur yang belum
            // pernah diuji kosong, sambil membawa boneka yang membuat
            // penampangnya jauh lebih besar daripada capit kosong.
            //
            // Posenya KORBAN_ANGKAT_*, bukan SIAP. SIAP dibentuk untuk
            // mendekat dengan capit kosong dan menganga; menempuhnya balik
            // sambil menggenggam mengayunkan pergelangan tanpa keperluan.
            //
            // BADAN PULANG DI FASE INI, satu fase sesudah capit menutup.
            // Bukan bersamaan dengan penutupan capit: badan yang mundur
            // selagi rahang masih menutup menarik boneka keluar dari capit
            // yang belum menggenggam. Bukan pula ditunda ke lipatan: selama
            // badan masih condong, ruas berikutnya berjalan dengan badan
            // miring ke depan di atas lantai pecah.
            //
            // X DAN ROTASI dinolkan di sini, termasuk X milik Raspi: ruas
            // berikutnya berjalan ke arah KAKI, dan badan yang masih menyerong
            // membuat langkah pertamanya miring.
            //
            // Y TIDAK pulang ke nol, ia justru MUNDUR ke
            // -KORBAN_ANGKAT_MUNDUR_MM. Diminta R2C 17 Sep 2026: boneka yang
            // terangkat sering menabrak dinding yang dihadapi robot, karena
            // busur lipatan ke REHAT jauh lebih gemuk daripada capit kosong
            // yang dipakai menyetel posenya. Mundurnya dilepas di fase 5,
            // sesudah lipatan selesai -- lihat di sana.
            //
            // Laju lengan diperlambat MULAI DI SINI -- genggamannya sudah
            // terjadi di fase 2, jadi gerakan pertama sambil membawa boneka
            // adalah yang ini. Tapi TIDAK selambat lipatan: lihat
            // LENGAN_SLEW_ANGKAT_DEG_S.
            robot.setSlewLengan(lengan, LENGAN_SLEW_ANGKAT_DEG_S);
            poseSendi(robot, lengan, KORBAN_ANGKAT_BAHU, KORBAN_ANGKAT_SIKU,
                      KORBAN_ANGKAT_PRG, "angkat");
            robot.setBodyTranslation(0.0f, -KORBAN_ANGKAT_MUNDUR_MM, 0.0f);
            robot.setBodyRotation(0.0f, 0.0f, 0.0f);
            break;
        case 4:   // Melipat ke REHAT, dan di sinilah korban DIGENDONG
            //    sepanjang jalan ke safe zone: terlipat di atas badan, bukan
            //    menjulur di depan. Lajunya diturunkan LAGI di sini, dari
            //    laju angkat ke laju lipat: ayunan bahu inilah yang
            //    benar-benar bisa melempar boneka keluar dari capit.
            robot.setSlewLengan(lengan, LENGAN_SLEW_REHAT_DEG_S);
            poseSendi(robot, lengan, REHAT_BAHU, REHAT_SIKU,
                      REHAT_PERGELANGAN, "rehat");
            break;
        case 5:   // Jatah KEDUA lipatan ANGKAT ke REHAT: bahu sendirian
            //    menempuh 90 der pada laju separuh, dan itu tidak muat dalam
            //    satu jatah. Angkanya diperiksa cek_lengan_laju.py --
            //    jalankan lagi kalau pose atau laju lengan disetel.
            //
            // BADAN PULANG DI SINI, melepas mundur fase 3. Ditunda sampai
            // sekarang karena dinding yang dihindari itu baru lewat setelah
            // boneka naik: melepasnya di fase 4 mengembalikan badan ke depan
            // tepat selagi busur lipatan masih di ketinggian dinding.
            //
            // Fase ini punya jatah penuh LENGAN_JEDA_MS dan tidak menyuruh
            // lengan apa-apa, jadi 20 mm pada laju condong (500 ms) lewat
            // dengan longgar. Laju badan dikembalikan ke BODY_SLEW_MM_S di
            // fase berikutnya, sesudah perjalanan pulang ini selesai.
            robot.setBodyTranslation(0.0f, 0.0f, 0.0f);
            break;
        default:
            robot.setSlewLengan(lengan, ARM_SLEW_DEG_S);
            // Laju geser badan dikembalikan DI SINI, bukan di fase 3: pose
            // badan baru saja dinolkan di sana, dan perjalanan pulang itu pun
            // harus pelan -- korban sudah ada di capit.
            robot.setBodySlewMm(BODY_SLEW_MM_S);
            return true;
    }
    return false;
}

// MENARUH, dicerminkan dari sekuensAmbil(). Diminta R2C 17 Sep 2026.
//
// Dua hal yang SENGAJA tidak ikut dicerminkan:
//
//   BADAN TIDAK PERNAH CONDONG. Fase translasi di AMBIL ada untuk satu
//   sebab: korban tertimbun reruntuhan, dan kaki tidak boleh maju ke sana.
//   Safe zone kosong dan datar -- tidak ada yang perlu dihindari, jadi
//   memajukan badan cuma menambah satu jatah waktu dan satu cara gagal.
//
//   TIDAK ADA VISION. Titik lepasnya ditentukan tabel, bukan kamera, dan
//   parkir vision di ruasMasuk() memang sudah dipagari `x.aksi == AKS_AMBIL`.
//
// Yang DICERMINKAN: turun lewat SIAP, bukan langsung ke titik lepas, lalu
// NAIK lagi sebelum melipat. Naik itu yang paling berharga di sini, dan
// sebabnya sudah tertulis di versi lama fungsi ini: capit lewat tepat di
// atas korban yang baru berdiri. Melipat dari titik lepas menyeret capit
// mendatar melintasi boneka; naik dulu membuatnya lewat di atasnya.
//
// Laju mengikuti BEBAN, bukan nomor fase: pelan selama masih menggenggam
// (fase 0 dan 1), penuh begitu capit terbuka (fase 3 dan 4).
static bool sekuensTaruh(Hexapod& robot, uint8_t lengan, uint8_t& langkah, uint32_t sejak) {
    uint8_t fase;
    if (!faseBaru(langkah, sejak, fase)) return false;
    switch (fase) {
        case 0:   // buka dari REHAT ke SIAP, capit MASIH menggenggam. Ayunan
            //    bahu inilah yang bisa melempar boneka, jadi laju lipat.
            robot.armEnable(lengan, true);
            robot.setSlewLengan(lengan, LENGAN_SLEW_REHAT_DEG_S);
            poseSendi(robot, lengan, KORBAN_SIAP_BAHU, KORBAN_SIAP_SIKU,
                      KORBAN_SIAP_PRG, "siap (menggendong)");
            break;
        case 1:   // KOSONG DENGAN SENGAJA -- jatah kedua untuk ayunan di atas.
            //    REHAT ke SIAP itu bahu 90 der, dan pada laju lipat ia makan
            //    2540 ms, lebih lama daripada satu LENGAN_JEDA_MS. Ketahuan
            //    cek_lengan_laju.py sebelum pernah dijalankan di robot.
            break;
        case 2:   // TURUN ke titik lepas. Siku yang bekerja, jadi boleh laju
            //    angkat -- sama seperti fase 4 AMBIL, arah terbalik.
            robot.setSlewLengan(lengan, LENGAN_SLEW_ANGKAT_DEG_S);
            poseSendi(robot, lengan, KORBAN_LEPAS_BAHU, KORBAN_LEPAS_SIKU,
                      KORBAN_LEPAS_PRG, "lepas");
            break;
        case 3:   // LEPAS HANYA sesudah sampai; melepas di tengah jalan berarti
            //    korban jatuh di luar safe zone dan nilainya hangus.
            robot.setGrip(lengan, KORBAN_GRIP_BUKA);
            break;
        case 4:   // NAIK menjauh dari korban yang baru berdiri, capit KOSONG.
            //    Tidak ada lagi yang bisa terlempar, jadi laju penuh.
            robot.setSlewLengan(lengan, ARM_SLEW_DEG_S);
            poseSendi(robot, lengan, KORBAN_ANGKAT_BAHU, KORBAN_ANGKAT_SIKU,
                      KORBAN_ANGKAT_PRG, "angkat (kosong)");
            break;
        case 5:   // melipat ke REHAT untuk ruas berikutnya
            poseSendi(robot, lengan, REHAT_BAHU, REHAT_SIKU,
                      REHAT_PERGELANGAN, "rehat");
            break;
        default:
            robot.setSlewLengan(lengan, ARM_SLEW_DEG_S);
            return true;
    }
    return false;
}

// ====================================================================

Misi::Misi(Hexapod& robot, Navigation& nav, LidarArray& lidar)
    : _robot(robot), _nav(nav), _lidar(lidar) {
    tabelBaku();     // isi RUAS[] dari flash, lalu _cm[] + hitungArah()
}

// ====================================================================
// EDITOR TABEL DARI HUD ('m5')
//
// Yang dijaga seluruh bagian ini cuma satu hal: tabel yang dijalankan saat
// percobaan harus sama dengan tabel yang nanti di-flash. Karena itu tiap
// perubahan memanggil tabelBerubah(), tiap perubahan ditolak selama misi
// berjalan, dan tabelCrc() memberi HUD cara memastikannya tanpa menebak.
// ====================================================================

void Misi::salinBaris(uint8_t ke, const Ruas& dari) {
    RUAS[ke] = dari;
    strncpy(NAMA_RAM[ke], dari.nama ? dari.nama : "", RUAS_NAMA_MAKS - 1);
    NAMA_RAM[ke][RUAS_NAMA_MAKS - 1] = '\0';
    RUAS[ke].nama = NAMA_RAM[ke];
}

void Misi::tabelBaku() {
    RUAS_N = RUAS_BAKU_N;
    for (uint8_t i = 0; i < RUAS_N; i++) salinBaris(i, RUAS_BAKU[i]);
    skorBaku();                       // peta poin ikut pulang ke flash
    tabelBerubah();
}

bool Misi::tabelSamaBaku() const {
    // Dibandingkan KOLOM PER KOLOM, bukan memcmp: `nama` sengaja menunjuk ke
    // kolam RAM, jadi pointernya memang berbeda dan memcmp akan selalu gagal.
    if (RUAS_N != RUAS_BAKU_N) return false;
    for (uint8_t i = 0; i < RUAS_N; i++) {
        const Ruas& a = RUAS[i];
        const Ruas& b = RUAS_BAKU[i];
        if (a.belok != b.belok || a.kemudi != b.kemudi || a.profil != b.profil
            || a.abaikanDepan != b.abaikanDepan || a.henti != b.henti
            || a.nilai != b.nilai || a.aksi != b.aksi || a.aksiA != b.aksiA
            || a.putar != b.putar || a.condong != b.condong
            || a.jagaBelakang != b.jagaBelakang || a.mundurMm != b.mundurMm
            || a.condongMm != b.condongMm) return false;
        if (strcmp(a.nama ? a.nama : "", b.nama ? b.nama : "") != 0) return false;
    }
    return true;
}

bool Misi::sisipBaris(uint8_t idx) {
    if (!bolehUbahTabel()) return false;
    if (RUAS_N >= RUAS_MAKS) {
        Serial.print("m5+: tabel penuh, batas RUAS_MAKS ");
        Serial.println(RUAS_MAKS);
        return false;
    }
    if (idx > RUAS_N) idx = RUAS_N;
    for (uint8_t i = RUAS_N; i > idx; i--) salinBaris(i, RUAS[i - 1]);
    // Kurung kosong WAJIB: anggota awal struct Ruas (nama..aksiA) tidak punya
    // nilai baku di deklarasinya, jadi `Ruas kosong;` berisi sampah stack --
    // dan sampah di kolom `henti` adalah ruas yang berjalan tanpa syarat henti.
    Ruas kosong{};
    kosong.nama = "baris baru";
    kosong.henti = HNT_LANGSUNG;       // baris kosong tidak boleh JALAN
    salinBaris(idx, kosong);
    RUAS_N++;
    skorSisip(idx);                    // peta poin ikut bergeser
    // Struktur berubah -> _cm[] disalin ulang seluruhnya, jadi override 'm7'
    // pada ruas mana pun hilang. Disengaja, dan dicetak supaya tidak senyap.
    tabelBerubah();
    Serial.print("m5+ "); Serial.print(idx);
    Serial.print(" -- "); Serial.print(RUAS_N);
    Serial.println(" baris. Panjang ruas hasil 'm7' dipulangkan ke tabel.");
    Serial.println("  Baris baru: HNT_LANGSUNG, tidak berjalan dan tidak berpoin.");
    return true;
}

bool Misi::hapusBaris(uint8_t idx) {
    if (!bolehUbahTabel()) return false;
    if (idx >= RUAS_N) {
        Serial.print("m5-: indeks "); Serial.print(idx);
        Serial.print(" di luar tabel 0.."); Serial.println(RUAS_N - 1);
        return false;
    }
    if (RUAS_N <= 1) {
        Serial.println("m5-: tabel tinggal satu baris, tidak dihapus.");
        return false;
    }
    for (uint8_t i = idx; i + 1 < RUAS_N; i++) salinBaris(i, RUAS[i + 1]);
    RUAS_N--;
    skorHapus(idx);
    tabelBerubah();
    Serial.print("m5- "); Serial.print(idx);
    Serial.print(" -- "); Serial.print(RUAS_N);
    Serial.println(" baris. Panjang ruas hasil 'm7' dipulangkan ke tabel.");
    return true;
}

void Misi::tabelBerubah() {
    // _cm[] adalah CACHE dari RUAS[].nilai, dan cache yang tidak diperbarui
    // adalah sebab nomor satu "sebelum flash bisa, sesudah flash tidak".
    for (uint8_t i = 0; i < RUAS_N; i++) _cm[i] = RUAS[i].nilai;
    // Arah mutlak tiap ruas turunan dari kolom `belok` + `putar`, jadi ia
    // WAJIB dihitung ulang walau cuma satu baris yang berubah: satu belok
    // yang bergeser memutar seluruh sisa lintasan.
    hitungArah();
}

bool Misi::bolehUbahTabel() {
    if (!berjalan()) return true;
    Serial.println("Tabel TIDAK diubah: misi sedang berjalan. Tekan 'm0' dulu.");
    return false;
}

bool Misi::setBaris(uint8_t idx, const float* v, const char* nama) {
    if (!bolehUbahTabel()) return false;
    if (idx >= RUAS_N) {
        Serial.print("m5s: indeks "); Serial.print(idx);
        Serial.print(" di luar tabel 0.."); Serial.println(RUAS_N - 1);
        return false;
    }
    Ruas& r = RUAS[idx];
    r.belok        = (Belok)  (uint8_t)v[0];
    r.kemudi       = (Kemudi) (uint8_t)v[1];
    r.profil       = (Profil) (uint8_t)v[2];
    r.abaikanDepan = v[3] > 0.5f;
    r.henti        = (Henti)  (uint8_t)v[4];
    r.nilai        = v[5];
    r.aksi         = (Aksi)   (uint8_t)v[6];
    r.aksiA        = (uint8_t)v[7];
    r.putar        = v[8];
    r.condong      = v[9]  > 0.5f;
    r.jagaBelakang = v[10] > 0.5f;
    r.mundurMm     = v[11];
    r.condongMm    = v[12];
    if (nama) {
        strncpy(NAMA_RAM[idx], nama, RUAS_NAMA_MAKS - 1);
        NAMA_RAM[idx][RUAS_NAMA_MAKS - 1] = '\0';
    }
    r.nama = NAMA_RAM[idx];
    // MENANG ATAS 'm7'. tabelBerubah() menyalin ulang seluruh _cm[] dari
    // tabel, jadi panjang ruas yang pernah disetel 'm7 <idx> <cm>' dipulangkan
    // ke angka tabel. Disengaja -- tabel RAM adalah satu-satunya sumber -- dan
    // diperingatkan di .ino supaya tidak senyap.
    tabelBerubah();
    return true;
}

uint16_t Misi::tabelCrc() const {
    // BENTUK KANONIK, dan ia harus sama byte per byte dengan
    // moses/lintasan.py::crc_tabel(). Kalau urutan di sini bergeser, CRC HUD
    // tidak akan pernah cocok lagi dan HUD mengirim ulang tabel tanpa henti --
    // gejalanya tidak akan terbaca sebagai "urutan byte berubah".
    uint8_t n = RUAS_N;
    uint16_t crc = Calib::crc16(&n, 1);
    for (uint8_t i = 0; i < RUAS_N; i++) {
        const Ruas& r = RUAS[i];
        uint8_t buf[25 + RUAS_NAMA_MAKS];
        uint8_t k = 0;
        buf[k++] = (uint8_t)r.belok;
        buf[k++] = (uint8_t)r.kemudi;
        buf[k++] = (uint8_t)r.profil;
        buf[k++] = r.abaikanDepan ? 1 : 0;
        buf[k++] = (uint8_t)r.henti;
        memcpy(buf + k, &r.nilai, 4);     k += 4;
        buf[k++] = (uint8_t)r.aksi;
        buf[k++] = r.aksiA;
        memcpy(buf + k, &r.putar, 4);     k += 4;
        buf[k++] = r.condong ? 1 : 0;
        buf[k++] = r.jagaBelakang ? 1 : 0;
        memcpy(buf + k, &r.mundurMm, 4);  k += 4;
        memcpy(buf + k, &r.condongMm, 4); k += 4;
        const char* s = r.nama ? r.nama : "";
        for (uint8_t j = 0; j < RUAS_NAMA_MAKS - 1 && s[j]; j++) buf[k++] = s[j];
        buf[k++] = 0;
        crc = Calib::crc16(buf, k, crc);
    }
    return crc;
}

void Misi::tabelVersi() {
    // Satu baris, dibaca HUD tiap kali ia perlu tahu apakah tabel RAM masih
    // sama dengan draft-nya. Sesudah reset Teensy, CRC kembali ke nilai tabel
    // flash -- dan justru itu yang membuat reset terdeteksi tanpa saklar
    // tambahan.
    Serial.print("#TABV "); Serial.print(tabelCrc());
    Serial.print(' ');      Serial.println(RUAS_N);
}

void Misi::tabelDump() {
    // Kolomnya URUTAN YANG SAMA dengan yang diterima 'm5s', jadi satu baris
    // '#TABR' bisa dikirim balik apa adanya sebagai 'm5s'. Angka pecahan
    // dicetak 2 desimal supaya putar/mundurMm/condongMm tidak dibulatkan.
    //
    // NILAI SUMBER, bukan hasil pencerminan: yang diedit adalah tabelnya, dan
    // belokRuas()/kemudiRuas() yang mencerminkannya saat dibaca. Ini kebalikan
    // dari tabel(), yang sengaja mencetak yang BERLAKU.
    for (uint8_t i = 0; i < RUAS_N; i++) {
        const Ruas& r = RUAS[i];
        Serial.print("#TABR ");         Serial.print(i);
        Serial.print(' '); Serial.print((uint8_t)r.belok);
        Serial.print(' '); Serial.print((uint8_t)r.kemudi);
        Serial.print(' '); Serial.print((uint8_t)r.profil);
        Serial.print(' '); Serial.print(r.abaikanDepan ? 1 : 0);
        Serial.print(' '); Serial.print((uint8_t)r.henti);
        Serial.print(' '); Serial.print(r.nilai, 2);
        Serial.print(' '); Serial.print((uint8_t)r.aksi);
        Serial.print(' '); Serial.print(r.aksiA);
        Serial.print(' '); Serial.print(r.putar, 2);
        Serial.print(' '); Serial.print(r.condong ? 1 : 0);
        Serial.print(' '); Serial.print(r.jagaBelakang ? 1 : 0);
        Serial.print(' '); Serial.print(r.mundurMm, 2);
        Serial.print(' '); Serial.print(r.condongMm, 2);
        Serial.print(" | ");
        Serial.println(r.nama ? r.nama : "");
    }
    tabelVersi();
}

// Belok relatif -> mata angin mutlak. Dulu rumus ini punya kembaran di
// Navigation::arahGeser(); kembarannya dihapus 6 Sep 2026 bersama
// belok-otomatis mode arena, jadi INILAH satu-satunya tempat arah mutlak
// dihitung di seluruh firmware.
// Serong yang terkumpul sejak _yawUjung dicatat, dibungkus ke +-180.
// Positif = badan berputar searah yaw naik; tandanya dicocokkan di arena
// sekali, lalu berlaku untuk semua cetakan ini.
float Misi::selisihYaw() const {
    float d = _nav.yawKini() - _yawUjung;
    while (d >= 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// --- ARENA CERMIN ---------------------------------------------------------
//
// Dibaca dari gParam, bukan disalin ke anggota: saklarnya boleh dibalik tombol
// D3 kapan saja, dan salinan yang lupa diperbarui berarti misi separuh
// tercermin. Harganya satu pembacaan float per pemakaian, dan itu tidak ada
// artinya dibanding sekali salah arah.
//
// hitungArah() dipanggil ulang tiap 'm1' dan 'm4', jadi membalik saklar lalu
// menjalankan misi sudah cukup -- tidak ada yang perlu dimuat ulang tangan.
bool Misi::arenaCermin() { return gParam[K_ARENA_MIRROR] >= 0.5f; }

Belok Misi::belokRuas(uint8_t i) const {
    const Belok b = RUAS[i].belok;
    if (!arenaCermin()) return b;
    // (4 - b) % 4: LURUS 0 tetap 0, KANAN 1 jadi 3 (KIRI), BALIK 2 tetap 2,
    // KIRI 3 jadi 1 (KANAN). Rumus, bukan tabel -- tabel kecil yang benar
    // hari ini adalah tabel yang salah begitu enumnya disisipi nilai baru.
    return (Belok)((4u - (uint8_t)b) % 4u);
}

Kemudi Misi::kemudiRuas(uint8_t i) const {
    const Kemudi k = RUAS[i].kemudi;
    if (!arenaCermin() || k == KMD_TENGAH) return k;   // TENGAH tidak berarah
    return (k == KMD_KIRI) ? KMD_KANAN : KMD_KIRI;
}

float Misi::putarRuas(uint8_t i) const {
    return arenaCermin() ? -RUAS[i].putar : RUAS[i].putar;
}

void Misi::hitungArah() {
    uint8_t a = MISI_ARAH_BERANGKAT;
    float   s = 0.0f;
    for (uint8_t i = 0; i < RUAS_N; i++) {
        a = (uint8_t)((a + (uint8_t)belokRuas(i)) % 4);
        // Serong menumpuk seperti belok menumpuk: keduanya "belok masuk ruas
        // ini", cuma satuannya berbeda. Yang satu dibungkus modulo 4, yang
        // satu dijumlahkan lurus.
        s += putarRuas(i);
        _arah[i]   = a;
        _serong[i] = s;
    }
}

// Heading MUTLAK ruas ini: mata angin arena + serong yang menumpuk.
// NAN bila mata anginnya belum dicatat kompas -- pemanggil harus memeriksa,
// karena seluruh misi mustahil tanpa itu.
float Misi::headingRuas(uint8_t i) const {
    const float dasar = _nav.headingArah(_arah[i]);
    if (isnan(dasar)) return NAN;
    float h = dasar + _serong[i];
    while (h >= 360.0f) h -= 360.0f;
    while (h < 0.0f)    h += 360.0f;
    return h;
}

// true bila ruas ini memang menyerong dari mata angin. Dipakai cetakan supaya
// "UTARA" tidak berbohong saat robot sebenarnya menghadap 45 der dari UTARA.
bool Misi::ruasSerong(uint8_t i) const { return fabsf(_serong[i]) > 0.5f; }

// Berapa derajat badan harus diputar supaya menghadap heading ruas ini.
// Positif/negatif mengikuti wrap180: hasilnya simpangan yang HARUS DIHABISKAN,
// bukan heading sasarannya.
//
// NAN dari headingRuas() atau IMU yang bisu -> 0, artinya "jangan luruskan".
// Menebak arah dengan angka yang tidak diketahui lebih buruk daripada tidak
// meluruskan sama sekali: yang pertama memutar badan ke arah yang salah tepat
// sebelum capit turun.
float Misi::koreksiYawRuas() const {
    const float sasar = headingRuas(_i);
    if (isnan(sasar)) return 0.0f;
    const float kini = _nav.yawKini();
    if (isnan(kini)) return 0.0f;
    float d = sasar - kini;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

void Misi::masuk(StatMisi s) {
    _stat = s;
    _t0 = millis();
    _n = 0;
    _stempel = 0;
    _langkah = 0;
    _tenangT0 = 0;
}

void Misi::gagal(const char* sebab) {
    _sebab = sebab;
    _stat  = MISI_GAGAL;
    lepasVisi();       // kamera tidak boleh ikut menggantung bersama misinya
    // Apa pun sebabnya, jangan tinggalkan robot berjalan buta ke depan.
    // navBerhenti() mengembalikan abaikanDepan sendiri, tapi ia menolak
    // bekerja kalau navigasi memang sudah diam -- dan justru itu jalur
    // kegagalan yang paling sering (navigasi berhenti duluan, misi menyusul).
    _nav.abaikanDepan(false);
    _nav.setTengah(false);
    Serial.print("\n!! MISI GAGAL di ruas "); Serial.print(_i);
    Serial.print(" ("); Serial.print(RUAS[_i].nama); Serial.println(")");
    Serial.print("   sebab: "); Serial.println(sebab);
    Serial.println("   Robot berhenti di tempat, servo TETAP HIDUP.");
    Serial.println("   'm' status  |  'm1' ulang dari awal  |  'm4 <idx>' lanjut dari ruas");
    Serial.println("   'x' melemaskan servo.");
}

// ====================================================================
// PEMERIKSAAN TABEL -- dijalankan sekali di 'm1', selagi robot masih diam.
//
// Semua yang diperiksa di sini adalah hal yang KALAU LOLOS akan merusak
// sesuatu di tengah arena, tempat tidak ada lagi yang bisa dilakukan.
// ====================================================================
bool Misi::tabelSiap(uint8_t dari, uint8_t sampai) {
    bool ok = true;

    for (uint8_t i = dari; i <= sampai; i++) {
        // 1) Ruas berjalan yang panjangnya belum diukur.
        if (RUAS[i].henti != HNT_LANGSUNG && _cm[i] < 0.0f) {
            if (ok) Serial.println("Gagal: masih ada ruas yang panjangnya BELUM DIUKUR.");
            Serial.print("  m7 "); Serial.print(i); Serial.print(" <cm>   -- ");
            Serial.println(RUAS[i].nama);
            ok = false;
        }

        // 2) Ruas yang berjalan BUTA ke depan tapi tidak dibatasi odometri.
        //    Kombinasi ini tidak punya apa pun yang menghentikannya: sensor
        //    depan dimatikan, dan HNT_DEPAN justru membaca sensor itu.
        //
        //    HNT_PUNCAK DIKECUALIKAN, 18 Sep 2026. 'buta ke depan' adalah
        //    setelan NAVIGATION -- ia mematikan tiga aturan kemudi supaya
        //    berkas yang menembak muka anak tangga tidak dibaca sebagai
        //    halangan. Yang membaca sensor di HNT_PUNCAK bukan Navigation
        //    melainkan ruasSelesai(), lewat _lidar.getDistance() langsung.
        //    Jadi di sana sensornya memang masih hidup, dan kombinasi ini
        //    justru yang dituju: buta untuk menyetir, melihat untuk berhenti.
        if (RUAS[i].abaikanDepan && RUAS[i].henti != HNT_ODO &&
            RUAS[i].henti != HNT_PUNCAK) {
            Serial.print("Gagal: ruas "); Serial.print(i);
            Serial.println(" berjalan buta ke depan tapi tidak dibatasi odometri.");
            Serial.println("  Perbaiki TABEL di Misi.cpp -- ini salah tulis, bukan salah setel.");
            ok = false;
        }

        // 3) Ambang sensor depan harus DI ATAS FRONT_STOP_CM. Di bawah angka
        //    itu navigasi keburu BERHENTI sendiri karena halangan depan, dan
        //    ruasnya tidak pernah sampai ke aksi ujungnya. (Dulu ia berbelok
        //    90 der alih-alih berhenti; belok-otomatisnya dihapus 6 Sep 2026,
        //    tapi ambangnya tetap harus di atas FRONT_STOP_CM.)
        // PUNCAK dengan nilai 0 SAH: itu artinya "gyro sendirian", bukan ambang
        // yang lupa diisi. HNT_DEPAN tidak punya arti itu -- di sana 0 memang
        // salah tulis.
        if ((RUAS[i].henti == HNT_DEPAN ||
             (RUAS[i].henti == HNT_PUNCAK && _cm[i] > 0.0f)) &&
            _cm[i] <= (float)FRONT_STOP_CM) {
            Serial.print("Gagal: ruas "); Serial.print(i);
            Serial.print(" ambang depan "); Serial.print(_cm[i], 0);
            Serial.print(" cm <= FRONT_STOP_CM "); Serial.println(FRONT_STOP_CM);
            Serial.println("  Navigasi akan berbelok sebelum misi sempat berhenti.");
            ok = false;
        }

        // 3b) Ruas GESER: sisi mana yang jadi penggaris dibaca dari kolom
        //    kemudi, jadi KMD_TENGAH tidak punya arti di sini. Dan sasarannya
        //    harus DI ATAS pita "terlalu dekat": penjaga arah geser memakai
        //    pita yang sama, sehingga sasaran di bawahnya akan ditolak tepat
        //    sebelum tercapai -- gagal di tengah arena, bukan di sini.
        if (RUAS[i].henti == HNT_SISI) {
            if (kemudiRuas(i) == KMD_TENGAH) {
                Serial.print("Gagal: ruas "); Serial.print(i);
                Serial.println(" bergeser tapi kemudinya MENENGAH -- pilih KANAN atau KIRI.");
                Serial.println("  Perbaiki TABEL di Misi.cpp -- ini salah tulis, bukan salah setel.");
                ok = false;
            }
            if (_cm[i] >= 0.0f && _cm[i] <= (float)WALL_MIN_CM) {
                Serial.print("Gagal: ruas "); Serial.print(i);
                Serial.print(" sasaran geser "); Serial.print(_cm[i], 0);
                Serial.print(" cm <= wall.min "); Serial.println(WALL_MIN_CM, 0);
                Serial.println("  Penjaga arah geser akan menolaknya sebelum sasaran tercapai.");
                ok = false;
            }
        }

        // 3c) Ruas MUNDUR: sasarannya harus DI ATAS pita "terlalu dekat",
        //    alasan yang sama persis dengan ruas GESER di atas.
        //    setelBelakangMulai() menolak sasaran <= WALL_MIN_CM, dan tanpa
        //    pemeriksaan ini penolakan itu datang di arena -- sesudah ruas
        //    sebelumnya selesai dan robot sudah berdiri di tempatnya.
        if (RUAS[i].henti == HNT_MUNDUR &&
            _cm[i] >= 0.0f && _cm[i] <= (float)MUNDUR_MIN_CM) {
            Serial.print("Gagal: ruas "); Serial.print(i);
            Serial.print(" sasaran mundur "); Serial.print(_cm[i], 0);
            Serial.print(" cm <= lantai mundur "); Serial.println(MUNDUR_MIN_CM, 0);
            Serial.println("  setelBelakangMulai() akan menolaknya sebelum berangkat.");
            ok = false;
        }

        // 3d) Ruas AMBIL dengan mundurMm: dua batas, dan keduanya diam-diam
        //    kalau tidak diperiksa di sini.
        //
        //    setBodyTranslation() meng-clamp ke BODY_MAX_TRANS_MM tanpa
        //    penanda apa pun, jadi mundur yang kejauhan berkurang sendiri dan
        //    busur turunnya tidak sebebas yang dikira.
        //
        //    Lalu fase BADAN MAJU menempuh mundurMm + KORBAN_CONDONG_MM dalam
        //    SATU jatah LENGAN_JEDA_MS. Lewat dari itu, fase berikutnya
        //    menimpanya di tengah jalan dan capit menutup sebelum badan
        //    sampai -- menggenggam udara di depan korban.
        //    SETIAP baris AMBIL diperiksa, bukan cuma yang mengisi kolomnya:
        //    translasi maju berlaku di semua AMBIL sejak 16 Sep 2026, dan
        //    mundur-saat-mengangkat sejak 17 Sep. Baris yang kolomnya kosong
        //    tetap menempuh KORBAN_CONDONG_MM, dan itu bisa melewati jatah
        //    sendirian kalau condong.mm disetel tinggi dari arena.
        if (RUAS[i].aksi == AKS_AMBIL) {
            // Kolom ruas menang atas param global, sama seperti di
            // sekuensAmbil(). Memeriksa yang global di sini sedangkan sekuens
            // memakai yang per-ruas berarti memeriksa angka yang salah.
            const float maju = (RUAS[i].condongMm > 0.0f) ? RUAS[i].condongMm
                                                          : KORBAN_CONDONG_MM;
            if (RUAS[i].mundurMm > BODY_MAX_TRANS_MM ||
                maju > BODY_MAX_TRANS_MM) {
                Serial.print("Gagal: ruas "); Serial.print(i);
                Serial.print(" mundur "); Serial.print(RUAS[i].mundurMm, 0);
                Serial.print(" / maju "); Serial.print(maju, 0);
                Serial.print(" mm > BODY_MAX_TRANS_MM "); Serial.println(BODY_MAX_TRANS_MM, 0);
                ok = false;
            }
            // DUA perjalanan badan, masing-masing satu jatah fase, dan yang
            // TERPANJANG yang menentukan:
            //   BADAN MAJU   dari -mundurMm ke +maju
            //   ANGKAT       dari +maju ke -KORBAN_ANGKAT_MUNDUR_MM
            // Memeriksa yang pertama saja pernah cukup; sejak mundur-saat-
            // mengangkat dipasang 17 Sep 2026, yang kedua bisa lebih panjang
            // kalau mundurMm kecil sementara maju besar.
            const float tempuhMaju   = RUAS[i].mundurMm + maju;
            const float tempuhAngkat = maju + KORBAN_ANGKAT_MUNDUR_MM;
            const float tempuh = (tempuhAngkat > tempuhMaju) ? tempuhAngkat
                                                             : tempuhMaju;
            const float muat   = KORBAN_CONDONG_LAJU_MM_S * (LENGAN_JEDA_MS / 1000.0f);
            if (tempuh > muat) {
                Serial.print("Gagal: ruas "); Serial.print(i);
                Serial.print(" badan menempuh "); Serial.print(tempuh, 0);
                Serial.print(" mm, hanya muat "); Serial.print(muat, 0);
                Serial.println(" mm dalam satu fase.");
                Serial.print("  maju "); Serial.print(tempuhMaju, 0);
                Serial.print(" mm, angkat "); Serial.print(tempuhAngkat, 0);
                Serial.println(" mm.");
                Serial.println("  Turunkan mundurMm/condong.mm, atau naikkan condong laju.");
                ok = false;
            }
        }

        // 3e) PUNCAK di ruas DATAR hampir pasti salah ketik. Bukan kegagalan:
        //    tanjakan bisa saja dilewati dengan profil lain suatu hari, dan
        //    menolaknya berarti menebak arena. Tapi HNT_PUNCAK menunggu badan
        //    MIRING dulu, dan di ruas datar itu tidak pernah terjadi -- ruasnya
        //    berjalan sampai batas waktu, tanpa satu pun pesan yang menyebut
        //    sebabnya.
        if (RUAS[i].henti == HNT_PUNCAK &&
            RUAS[i].profil != PRF_TANJAK && RUAS[i].profil != PRF_KAIL) {
            Serial.print("Awas: ruas "); Serial.print(i);
            Serial.println(" HNT_PUNCAK tapi profilnya bukan TANJAK/KAIL.");
            Serial.println("  PUNCAK menunggu badan MIRING dulu. Di ruas datar itu");
            Serial.println("  tidak pernah terjadi, dan ruasnya habis di batas waktu.");
        }

        // 3f) PUNCAK dengan nilai 0 tidak punya jalan keluar kalau IMU bisu.
        //    Peringatan, bukan penolakan: IMU yang sehat membuat kombinasi ini
        //    benar dan memang diminta di R-9. Yang dijaga cuma supaya orang
        //    tahu bahwa satu-satunya penahan tersisa adalah batas waktu ruas.
        if (RUAS[i].henti == HNT_PUNCAK && _cm[i] <= 0.0f) {
            Serial.print("Awas: ruas "); Serial.print(i);
            Serial.println(" HNT_PUNCAK nilai 0 -- dinding depan dimatikan, gyro sendirian.");
            Serial.println("  Kalau IMU bisu, TIDAK ADA yang mengakhiri ruas ini selain");
            Serial.println("  batas waktu. Periksa 'k' dan IMU sebelum berangkat.");
        }

        // 4) Pemicu sensor belakang harus jatuh di dalam jangkauan sensor.
        //    Di luar itu bacaannya jatuh ke LIDAR_JAUH dan pemicunya tidak
        //    pernah menyala -- robot melewati sasarannya lalu mati kehabisan
        //    waktu, tanpa satu pun petunjuk sebabnya.
        if (RUAS[i].henti == HNT_BELAKANG && _cm[i] >= (float)LIDAR_MAX_CM - 15.0f) {
            Serial.print("Gagal: ruas "); Serial.print(i);
            Serial.print(" jarak tempuh "); Serial.print(_cm[i], 0);
            Serial.print(" cm terlalu jauh untuk sensor belakang (maks ");
            Serial.print(LIDAR_MAX_CM); Serial.println(" cm).");
            ok = false;
        }
    }

    // 5) Setiap capit hanya muat SATU korban. Disimulasikan sepanjang tabel
    //    dari ruas 0, karena inilah satu-satunya cacat di sini yang tidak
    //    kelihatan dari satu baris saja -- ia lahir dari URUTAN.
    //
    //    Tabel 6 Sep 2026 melanggarnya: K-2 diambil di ruas 8 dan baru ditaruh
    //    jauh sesudahnya, sementara K-3 dan K-4 diambil di antaranya. Tiga
    //    korban sekaligus di dua lengan, dan semuanya ditulis ARM_DEPAN.
    //    Tanpa pemeriksaan ini, misi berjalan dan capitnya diam-diam menimpa
    //    korban yang sedang dipegang.
    // Disimulasikan dari `dari`, bukan dari 0, dengan kedua capit KOSONG --
    // itu memang keadaan robot saat mulaiDari() menolkan _korban.
    bool isi[2] = { false, false };
    for (uint8_t i = dari; i <= sampai; i++) {
        if (RUAS[i].aksi == AKS_TIDAK_ADA) continue;
        const uint8_t a = RUAS[i].aksiA & 1;
        const char* nama = a == ARM_DEPAN ? "DEPAN" : "BELAKANG";
        if (RUAS[i].aksi == AKS_AMBIL) {
            // SESUATU harus menaruh korban di dalam amplop jangkauan lengan
            // (lihat cek_korban.cpp), karena sekuensnya sendiri buta: ia
            // memainkan sudut sendi TETAP dan tidak pernah bertanya di mana
            // korbannya. Baris AMBIL yang berhenti di jarak sembarang
            // membuat lengan meraih udara tanpa satu pun pesan.
            //
            // Ada DUA cara yang sah, dan sejak 17 Sep 2026 keduanya dipakai:
            //
            //   HNT_DEPAN     ruas ini sendiri yang berjalan sampai LiDAR
            //                 depan membaca `nilai`.
            //   HNT_LANGSUNG  ruas ini tidak berjalan; ruas SEBELUMNYA yang
            //                 sudah menempatkan robot. Dipakai K-1 dan K-2,
            //                 yang didahului ruas HNT_MUNDUR ke tembok
            //                 belakang -- penggaris yang lebih jujur daripada
            //                 LiDAR depan di ceruk sempit, yang sering
            //                 membaca dinding seberang, bukan boneka.
            //
            // Yang TIDAK sah: HNT_LANGSUNG tanpa ruas pendekat di depannya.
            // K-3, K-4 dan K-5 didahului HNT_SISI (menyamping) atau baris
            // TARUH (tidak bergerak), jadi di sana jalan maju itulah satu-
            // satunya pendekatan. Mengubahnya jadi HNT_LANGSUNG membuat
            // capit menutup di udara, dan itu tidak kelihatan dari tabel.
            if (RUAS[i].henti != HNT_DEPAN && RUAS[i].henti != HNT_LANGSUNG) {
                Serial.print("Gagal: ruas "); Serial.print(i);
                Serial.println(" mengambil korban tapi hentinya bukan HNT_DEPAN atau HNT_LANGSUNG.");
                Serial.println("  HNT_DEPAN <cm> = ruas ini yang mendekat.");
                Serial.println("  HNT_LANGSUNG   = ruas sebelumnya yang sudah menempatkan robot.");
                ok = false;
            }
            // PERINGATAN, BUKAN PENOLAKAN. Diputuskan R2C 17 Sep 2026.
            //
            // Yang benar tidak bisa dibaca dari tabel: jarak boleh saja sudah
            // diatur ruas yang lebih jauh ke belakang, atau oleh operator yang
            // menempatkan robot dengan tangan sebelum start. Menolak baris
            // seperti itu memaksa menulis ruas palsu supaya lolos, dan ruas
            // palsu lebih berbahaya daripada peringatan yang dibaca.
            //
            // Tetap dicetak karena akibatnya diam: kalau tebakannya meleset,
            // capit menutup di udara tanpa satu pun pesan. Baris ini yang
            // mengingatkan ke mana harus melihat waktu itu terjadi.
            if (RUAS[i].henti == HNT_LANGSUNG) {
                const bool adaPendekat = (i > 0)
                    && (RUAS[i - 1].henti == HNT_MUNDUR
                        || RUAS[i - 1].henti == HNT_DEPAN);
                if (!adaPendekat) {
                    Serial.print("Awas: ruas "); Serial.print(i);
                    Serial.println(" AMBIL HNT_LANGSUNG tanpa ruas pendekat di depannya.");
                    Serial.println("  Jaraknya diatur dari luar tabel. Kalau capit menutup di");
                    Serial.println("  udara, ini tempat pertama yang harus diperiksa: HNT_SISI");
                    Serial.println("  cuma menggeser menyamping, baris TARUH tidak bergerak.");
                }
            }
            if (isi[a]) {
                Serial.print("Gagal: ruas "); Serial.print(i);
                Serial.print(" mengambil korban dengan capit "); Serial.print(nama);
                Serial.println(" yang MASIH TERISI.");
                Serial.println("  Taruh dulu, atau pindahkan ruas ini ke capit sebelahnya.");
                ok = false;
            }
            isi[a] = true;
        } else {
            if (!isi[a]) {
                Serial.print("Gagal: ruas "); Serial.print(i);
                Serial.print(" menaruh korban dari capit "); Serial.print(nama);
                Serial.println(" yang KOSONG.");
                ok = false;
            }
            isi[a] = false;
        }
    }
    return ok;
}

// ====================================================================

void Misi::mulai() { mulaiDari(0); }

// Syarat yang WAJIB benar sebelum robot boleh bergerak sama sekali. Dipakai
// bersama oleh 'm1'/'m4' dan mode ukur 'm6' -- satu pemeriksaan, satu pesan.
bool Misi::siapJalan(uint8_t idx, uint8_t sampai) {
    if (berjalan()) {
        Serial.println("Misi sedang berjalan. 'm' status, 'm0' batalkan dulu.");
        return false;
    }
    if (idx >= RUAS_N) {
        Serial.print("Ruas "); Serial.print(idx); Serial.print(" tidak ada -- hanya 0..");
        Serial.println(RUAS_N - 1);
        return false;
    }

    // navMulai() SUDAH memeriksa servo, mux, sensor depan, sensor samping,
    // IMU, kelengkapan kompas, dan kalibrasi pivot -- lalu mencetak persis
    // mana yang gagal. Yang diperiksa DI SINI hanya dua hal yang kalau
    // dibiarkan akan gagal SESUDAH robot terlanjur berputar 20 detik:
    // pivotKe() cuma MEMPERINGATKAN kalau arah putar belum dikalibrasi,
    // sedangkan mode arena MENOLAK.
    if (!_nav.kompasLengkap()) {
        Serial.println("Gagal: kompas arena belum lengkap -- 'c0'..'c3' lalu 'e', atau 'E'.");
        Serial.println("       Periksa dengan 'k'.");
        return false;
    }
    if (!_nav.pivotTerkalibrasi()) {
        Serial.println("Gagal: pivot belum dikalibrasi -- 'C' lalu 'S' sekali saja.");
        Serial.println("       Periksa dengan 'K'. Tanpa itu arah putar masih tebakan.");
        return false;
    }

    // Sensor BELAKANG adalah satu-satunya yang bisa melihat K-1 dilewati:
    // korbannya duduk di ceruk DI SAMPING lintasan (guidebook hal. 25, ruang
    // lebar 40 cm kedalaman 15 cm), jadi sensor depan tidak akan pernah
    // melihatnya. navMulai() tidak memeriksa sensor ini karena navigasi
    // memang tidak memakainya.
    // Sensor SISI yang jadi penggaris ruas geser. Sama alasannya: kalau ia
    // mati, ratakanMulai() menolak -- tapi menolaknya di tengah arena, sesudah
    // robot terlanjur menempuh belasan ruas.
    // SAMPAI `sampai`, bukan sampai akhir tabel. Lari sebagian ('m4 17 17'
    // untuk latihan di K-3) tidak boleh menuntut sensor yang cuma dipakai
    // ruas yang memang tidak akan dijalankan -- itu membuat latihan satu ruas
    // ditolak karena kanal LiDAR yang tidak ada hubungannya.
    for (uint8_t i = idx; i <= sampai && i < RUAS_N; i++) {
        if (RUAS[i].henti != HNT_SISI) continue;
        const uint8_t ch = (kemudiRuas(i) == KMD_KIRI) ? LIDAR_KIRI_D : LIDAR_KANAN_D;
        if (_lidar.getDistance(ch) != LIDAR_MATI) continue;
        Serial.print("Gagal: sensor "); Serial.print(LidarArray::nama(ch));
        Serial.print(" mati, padahal ruas "); Serial.print(i);
        Serial.println(" memakainya sebagai penggaris geser.");
        Serial.println("       'I' untuk init ulang, lalu 'l' untuk memastikan.");
        return false;
    }

    bool perluBelakang = false;
    for (uint8_t i = idx; i <= sampai && i < RUAS_N; i++)
        if (RUAS[i].henti == HNT_BELAKANG ||
            RUAS[i].henti == HNT_MUNDUR) perluBelakang = true;
    if (perluBelakang && _lidar.getDistance(LIDAR_BACK) == LIDAR_MATI) {
        Serial.print("Gagal: sensor BELAKANG (channel "); Serial.print(LIDAR_BACK);
        Serial.println(") tidak merespons.");
        Serial.println("       Dialah yang mengukur jarak tempuh dari dinding START.");
        Serial.println("       'I' untuk init ulang, lalu 'l' untuk memastikan.");
        return false;
    }
    return true;
}

void Misi::mulaiDari(uint8_t idx, uint8_t sampai) {
    if (sampai >= RUAS_N) sampai = RUAS_N - 1;
    if (!siapJalan(idx, sampai)) return;
    if (sampai < idx) {
        Serial.print("Ruas akhir "); Serial.print(sampai);
        Serial.print(" ada SEBELUM ruas awal "); Serial.println(idx);
        return;
    }
    if (!tabelSiap(idx, sampai)) return;
    _iAkhir = sampai;

    // Mulai dari keadaan yang DIKETAHUI. Kalau lari sebelumnya gagal di
    // tengah ruas kasar, profil gait masih TANGGA dan kemudi masih MENENGAH;
    // tanpa baris ini lari berikutnya menyusuri lorong datar dengan siklus
    // dua kali lipat dan kehabisan waktu tanpa sebab yang kelihatan.
    _robot.profileFlat();
    _nav.setTengah(false);
    _nav.abaikanDepan(false);

    _i       = idx;
    _blkAwal = -1.0f;
    _korban[0] = _korban[1] = false;
    _parkirVision = false;
    _visiJalan    = false;
    _sebab   = nullptr;
    _serongT0 = 0;
    _lidarUlang = 0;

    Serial.println("\n=== MISI MULAI ===");
    Serial.print("  ruas "); Serial.print(idx); Serial.print(" dari 0..");
    Serial.print(RUAS_N - 1); Serial.print("  --  "); Serial.println(RUAS[idx].nama);
    Serial.print("  berangkat   : "); Serial.print(_nav.namaArah(MISI_ARAH_BERANGKAT));
    Serial.println("  <- c0, arah LORONG PERTAMA dari HOME.");
    Serial.println("                Salah catat di sini menggeser SELURUH kolom arah.");
    Serial.print("  Batas waktu kontes "); Serial.print(MISI_TOTAL_BATAS_MS / 1000);
    Serial.println(" detik. 's', 'x', Enter, atau 'm0' membatalkan kapan saja.");
    Serial.println("  Capit belum terpasang: tiap ruas korban hanya BERHENTI sejenak.");

    _tMisi = millis();
    ruasMasuk();
}

// ====================================================================
// MODE UKUR -- yang menghasilkan angka untuk tabel.
//
// Robot sudah punya odometer; mengetik ulang angkanya dari meteran cuma
// menambah satu tempat untuk salah. Mode ini menjalankan SATU ruas dengan
// profil, kemudi, dan arah miliknya sendiri, lalu berjalan terus sampai
// operator menghentikannya di ujung ruas. Saat berhenti, robot mencetak
// sendiri berapa cm yang ditempuhnya.
//
// Profil gait ikut dipakai dengan sengaja: langkah TANGGA dan MERUNDUK
// panjangnya berbeda dari DATAR, jadi mengukur ruas lantai pecah dengan
// profil datar menghasilkan angka yang tidak berlaku saat ruas itu benar-
// benar dijalani.
// ====================================================================
void Misi::ukur(uint8_t idx) {
    if (!siapJalan(idx, idx)) return;

    const Ruas& x = RUAS[idx];
    if (x.henti == HNT_LANGSUNG) {
        Serial.print("Ruas "); Serial.print(idx);
        Serial.println(" tidak berjalan (hanya aksi) -- tidak ada yang bisa diukur.");
        return;
    }
    if (x.henti == HNT_SISI) {
        Serial.print("Ruas "); Serial.print(idx);
        Serial.println(" bergeser menyamping -- yang disetel bukan jarak tempuh.");
        Serial.println("  Sasarannya jarak ke dinding SAMPING: baca dari 'l', lalu 'm7'.");
        Serial.println("  Cobanya dengan 'V<cm>' langsung, tanpa lewat misi.");
        return;
    }
    if (x.henti == HNT_MUNDUR) {
        Serial.print("Ruas "); Serial.print(idx);
        Serial.println(" berjalan MUNDUR -- yang disetel bukan jarak tempuh.");
        Serial.println("  Sasarannya jarak MUTLAK ke dinding belakang: baca dari 'l', lalu 'm7'.");
        Serial.println("  Cobanya dengan 'J<cm>' langsung, tanpa lewat misi.");
        return;
    }
    if (x.henti == HNT_DEPAN) {
        Serial.print("Ruas "); Serial.print(idx);
        Serial.println(" berhenti pada DINDING, bukan pada jarak tempuh.");
        Serial.println("  Yang disetel di sana bukan panjang ruas melainkan seberapa dekat");
        Serial.println("  robot boleh mendekat. Bacanya dari 'l' (LiDAR depan), bukan dari sini.");
        return;
    }

    _robot.profileFlat();
    _nav.setTengah(false);
    _nav.abaikanDepan(false);

    _i        = idx;
    _blkAwal  = -1.0f;
    _sebab    = nullptr;
    _serongT0 = 0;
    _tMisi    = millis();

    Serial.print("\n=== MODE UKUR -- ruas "); Serial.print(idx);
    Serial.print(": "); Serial.println(x.nama);
    Serial.print("  berangkat "); Serial.print(_nav.namaArah(MISI_ARAH_BERANGKAT));
    Serial.println(" = c0 = arah lorong pertama dari HOME");
    Serial.print("  arah "); Serial.print(_nav.namaArah(_arah[idx]));
    if (ruasSerong(idx)) { Serial.print(" serong "); Serial.print(_serong[idx], 0);
                           Serial.print(" der (heading "); Serial.print(headingRuas(idx), 0);
                           Serial.print(")"); }
    const Kemudi kmd = kemudiRuas(_i);
    Serial.print(", kemudi "); Serial.print(kmd == KMD_TENGAH ? "MENENGAH" :
                                            kmd == KMD_KIRI   ? "dinding KIRI" : "dinding KANAN");
    Serial.print(", profil ");
    Serial.println(x.profil == PRF_TANGGA   ? "TANGGA" :
                   x.profil == PRF_MERUNDUK ? "MERUNDUK" :
                   x.profil == PRF_SEMPIT   ? "SEMPIT" :
                   x.profil == PRF_KAIL     ? "KAIL" :
                   x.profil == PRF_TANJAK   ? "TANJAK" : "DATAR");
    Serial.println("  TIDAK ada syarat henti -- robot berjalan sampai ANDA menghentikannya.");
    Serial.println("  Hentikan tepat di ujung ruas: 's', Enter, atau 'm0'.");
    if (x.abaikanDepan)
        Serial.println("  ! Sensor depan DIABAIKAN di ruas ini -- robot buta ke depan, awasi.");

    _ukurMulai = MISI_UKUR;   // ditandai supaya ruasMasuk() berhenti di MISI_UKUR
    ruasMasuk();
}

void Misi::ujiLengan(bool ambil) {
    if (berjalan()) {
        Serial.println("Misi/sekuens lain sedang berjalan -- 'm0' dulu.");
        return;
    }
    if (!_robot.isArmed()) { Serial.println("Servo masih lemas -- ketik 'b' dulu."); return; }

    _uji      = true;
    _ujiAmbil = ambil;
    masuk(MISI_LENGAN);          // masuk() yang menyetel _t0 dan _langkah

    Serial.print("\n=== SEKUENS LENGAN ");
    Serial.print(ambil ? "AMBIL" : "TARUH");
    Serial.println(" (uji, lepas dari tabel) ===");
    Serial.print("  tiap pose ");
    Serial.print(LENGAN_JEDA_MS);
    Serial.println(" ms, servo merayap 120 der/detik. 'm0' atau 's' untuk berhenti.");
}

// TUTUP JENDELA DETEKSI DI RASPI. '#LEPAS' adalah satu-satunya token yang
// sudah dikenal sisi Pi sebagai "selesai, berhentilah" -- memakainya berarti
// pembatalan bekerja tanpa Pi perlu diperbarui lebih dulu.
//
// Dipanggil dari batal() DAN gagal(). Sebelum ini keduanya meninggalkan
// kamera menyala: '#KORBAN' sudah terkirim, '#LEPAS' hanya dikirim di jalur
// SUKSES, jadi misi yang dihentikan di depan korban membuat Raspi mendeteksi
// terus tanpa ada yang akan menjawabnya.
void Misi::lepasVisi() {
    if (!_visiJalan) return;
    _visiJalan    = false;
    _parkirVision = false;
    KORBAN_SERIAL.print("#LEPAS ");
    KORBAN_SERIAL.println(_i);
}

void Misi::batal(const char* alasan) {
    _uji = false;
    lepasVisi();
    // Pola sama dengan navBerhenti(): kalau tidak ada yang berjalan, jangan
    // mencetak apa pun -- tanpa ini tiap 'x' dan Enter meninggalkan baris
    // "Misi DIBATALKAN" walau tak ada misi sama sekali.
    if (!berjalan()) { _stat = MISI_DIAM; return; }

    // Mode ukur: inilah momennya -- operator berhenti di ujung ruas, dan
    // jarak tempuh sejak ruas dimulai ADALAH panjang ruas itu. Dicetak dengan
    // penanda tetap supaya alat di PC bisa memungutnya tanpa menebak.
    bool sedangUkur = (_stat == MISI_UKUR);
    float tempuh = _robot.jarakCm() - _ruasAwal;

    _stat = MISI_DIAM;
    _ukurMulai = MISI_DIAM;
    _sebab = nullptr;
    _nav.navBerhenti(alasan);
    // navBerhenti() memulihkan abaikanDepan tapi TIDAK tengah, dan ia diam
    // saja kalau navigasi memang sudah berhenti duluan. Bereskan sendiri.
    _nav.abaikanDepan(false);
    _nav.setTengah(false);
    if (sedangUkur) {
        Serial.print("\nHASIL UKUR ruas "); Serial.print(_i);
        Serial.print(" = "); Serial.print(tempuh, 1); Serial.println(" cm");
        Serial.print("UKUR "); Serial.print(_i); Serial.print(" ");
        Serial.println(tempuh, 1);          // baris tetap untuk alat di PC
        Serial.print("  Simpan dengan: m7 "); Serial.print(_i); Serial.print(" ");
        Serial.println((int)(tempuh + 0.5f));
        return;
    }

    Serial.print("Misi DIBATALKAN di ruas "); Serial.print(_i);
    Serial.print(" ("); Serial.print(RUAS[_i].nama); Serial.print(")");
    if (alasan) { Serial.print(": "); Serial.println(alasan); } else Serial.println(".");
}

// ====================================================================
// MASUK RUAS: pivot dulu bila arahnya berbeda, baru berjalan.
// ====================================================================
void Misi::ruasMasuk() {
    const Ruas& x = RUAS[_i];

    Serial.print("\n--- RUAS "); Serial.print(_i); Serial.print(": ");
    Serial.print(x.nama); Serial.print("  ["); Serial.print(_nav.namaArah(_arah[_i]));
    if (ruasSerong(_i)) { Serial.print(" serong "); Serial.print(_serong[_i], 0); Serial.print(" der"); }
    Serial.println("]");

    // Badan sudah menghadap arah ruas ini? Kalau belum, putar dulu. Ini juga
    // yang menangani ketentuan mulai kontes: robot boleh diletakkan menghadap
    // ke mana saja, dan ruas 0 memaksanya menghadap arah berangkat sebelum
    // mode arena sempat mengunci mata angin yang salah.
    const float sasar = headingRuas(_i);
    if (isnan(sasar)) {
        lewati("mata angin ruas ini belum dicatat kompas -- 'c0'..'c3' lalu 'e', atau 'E'.");
        return;
    }
    if (!_nav.diHeading(sasar)) {
        _pivotUlang = 0;            // jatah ulang, dihitung per RUAS bukan per pivot
        _nav.pivotKe(sasar);
        if (!_nav.pivotSedangJalan()) {
            lewati("pivot masuk ruas tidak mau jalan -- sebabnya tercetak di atas ('b' bila servo lemas).");
            return;
        }
        masuk(MISI_PIVOT);
        return;
    }

    ruasBerangkat();
}

// Profil gait ruas ini, plus jawaban atas satu pertanyaan: boleh langsung
// berangkat, atau badan harus tenang dulu?
//
// HANYA ruas yang berhenti pada SENSOR yang menunggu. Ruas HNT_ODO tidak,
// dan itu disengaja: aturan "ruas beruntun disambung tanpa berhenti" ada
// untuk menjaga robot tidak tersendat di bibir rintangan, dan odometri tidak
// peduli berkas sensor sedang berayun. Yang tidak bisa ditawar cuma pemicu
// jarak -- ia membaca satu angka lalu mengakhiri ruas di situ.
//
// Kemudi dinding samping juga membaca LiDAR selama tunggu ini, tapi ia
// MENGOREKSI terus-menerus: satu sampel miring diperbaiki sampel berikutnya.
// Pemicu tidak punya kesempatan kedua.
bool Misi::pasangProfil() {
    const Ruas& x = RUAS[_i];

    switch (x.profil) {
        case PRF_TANGGA:   _robot.profileStairs(); break;
        case PRF_MERUNDUK: _robot.profileCrouch(); break;
        case PRF_SEMPIT:   _robot.profileNarrow(); break;
        case PRF_KAIL:     _robot.profileKail();   break;
        case PRF_TANJAK:   _robot.profileTanjak(); break;
        default:           _robot.profileFlat();   break;
    }

    // KELUAR DARI TANJAK: lengan dipulangkan ke pose REHAT, sama dengan 'R'.
    // Bentuk TANJAK melipat lengan ke atas supaya tidak menyapu anak tangga;
    // di ruas datar lipatan itu tidak ada gunanya, dan boneka yang digendong
    // duduk di REHAT. Diminta R2C 18 Sep 2026, DI MISI SAJA -- 'T0'..'T4'
    // manual tidak menyentuh lengan, supaya penyetelan lengan dengan tangan
    // tidak terhapus tiap ganti profil.
    //
    // PRF_SEMPIT ikut dikecualikan: ia memakai bentuk TANJAK, jadi ia sudah
    // memasang pose lengannya sendiri dan REHAT di sini akan menimpanya.
    if (_i > 0 && RUAS[_i - 1].profil == PRF_TANJAK &&
        x.profil != PRF_TANJAK && x.profil != PRF_SEMPIT &&
        _robot.isArmed()) {
        _robot.armEnable(ARM_DEPAN, true);
        _robot.setSudutLengan(ARM_DEPAN, REHAT_BAHU, REHAT_SIKU,
                                         REHAT_PERGELANGAN, nullptr);
        Serial.println("  lengan dipulangkan ke pose REHAT (keluar dari TANJAK).");
    }

    bool pakaiSensor = (x.henti == HNT_DEPAN ||
                        x.henti == HNT_BELAKANG ||
                        x.henti == HNT_SISI ||
                        x.henti == HNT_MUNDUR ||
                        x.henti == HNT_PUNCAK);
    if (!pakaiSensor) { _pivotBaru = false; return false; }

    // SESUDAH PIVOT, TUNGGU JUGA. Diminta R2C 17 Sep 2026.
    //
    // Urutan pivot-lalu-baca memang sudah benar: ruasMasuk() memutar badan
    // dan baru memanggil ruasBerangkat() sesudah MISI_PIVOT selesai. Yang
    // kurang bukan urutannya, melainkan UMUR SAMPELNYA.
    //
    // LiDAR berputar terus selama pivot, dan sampel yang tersimpan saat pivot
    // berakhir lahir waktu badan masih menghadap ke arah lain. Pemicu jarak
    // membaca satu angka lalu mengakhiri ruas di situ -- ia tidak punya
    // kesempatan kedua, tidak seperti kemudi dinding yang mengoreksi terus.
    // Satu sampel basah dari heading lama sudah cukup mengakhiri ruas di
    // tempat yang salah.
    //
    // Yang ditunggu MISI_LIDAR_SEGAR_MS, sama seperti tunggu profil: cukup
    // untuk seluruh berkas sensor berganti isi tiga kali.
    if (_pivotBaru) {
        _pivotBaru = false;
        _nav.navBerhenti("menunggu sampel LiDAR yang lahir SESUDAH pivot.");
        return true;
    }
    if (_robot.gaitProfilTenang()) return false;

    // BERHENTI DULU. Ruas beruntun disambung tanpa menghentikan navigasi,
    // jadi tanpa baris ini robot menunggu sambil TETAP BERJALAN -- 1,2 detik
    // dengan pemicu jarak yang belum boleh dipercaya berarti ~15 cm terlewat,
    // persis kesalahan yang mau dicegah.
    _nav.navBerhenti("menunggu profil gait tenang sebelum membaca LiDAR.");
    return true;
}

// Berangkat menjalani RUAS[_i]: pasang profil, tunggu badan tenang bila ruas
// ini membaca sensor, baru jalan. Dipanggil dari tiga tempat -- masuk ruas
// tanpa pivot, sesudah pivot selesai, dan sesudah tunggu selesai. Yang
// terakhir aman berulang: profilnya sudah terpasang, jadi pasangProfil()
// menjawab "tidak perlu menunggu" dan jalur ini lewat sekali jalan.
void Misi::ruasBerangkat() {
    if (pasangProfil()) {
        masuk(MISI_SETEL);
        // Sebabnya sudah dicetak navBerhenti() di dalam pasangProfil(), jadi
        // di sini cukup menyebut SENSOR MANA yang ditunggu.
        Serial.print("  menunggu sebelum ");
        Serial.println(RUAS[_i].henti == HNT_SISI ? "membaca dinding samping."
                                                  : "membaca LiDAR.");
        return;
    }
    if (!ruasJalan()) return;
    masuk(_ukurMulai == MISI_UKUR ? MISI_UKUR : MISI_JALAN);
}

bool Misi::ruasJalan() {
    const Ruas& x = RUAS[_i];

    _nav.setTengah(kemudiRuas(_i) == KMD_TENGAH);
    _nav.abaikanDepan(x.abaikanDepan);

    // Ruas yang cuma aksi: tidak ada yang perlu dijalankan.
    if (x.henti == HNT_LANGSUNG) return true;

    // Ruas GESER: bukan mode arena sama sekali. Navigation yang menutup
    // lupnya sendiri (syarat henti dibaca tiap tick), jadi di sini cukup
    // menyalakannya dan menunggu -- ruasSehat()/ruasSelesai() punya cabang
    // sendiri untuk membaca hasilnya.
    if (x.henti == HNT_SISI) {
        if (!_nav.ratakanMulai(kemudiRuas(_i) == KMD_KIRI, (int)_cm[_i], x.jagaBelakang)) {
            lewati("perataan sisi ditolak -- sebabnya tercetak di atas.");
            return false;
        }
        _ruasAwal = _robot.jarakCm();
        _serongT0 = 0;
        return true;
    }

    // Ruas MUNDUR: sama polanya dengan GESER di atas. Navigation menutup
    // lupnya sendiri lewat setelBelakangUpdate(), jadi di sini cukup
    // menyalakannya. Mode arena TIDAK dinyalakan -- robot berjalan mundur,
    // dan kemudi dinding yang menghadap ke depan tidak punya arti di sana.
    if (x.henti == HNT_MUNDUR) {
        // TIDAK ADA DINDING DI BELAKANG: ruas ini dilewati, bukan dijalankan.
        // Sasaran mundur adalah BACAAN sensor belakang, jadi tanpa dinding di
        // dalam jangkauan tidak ada yang bisa dituju -- robot akan mundur
        // sampai batas waktu, menjauhi lintasan. Diminta R2C 18 Sep 2026 untuk
        // baris "mundur jika bisa": ruas ini memang oportunistik.
        //
        // LIDAR_MATI TIDAK ikut ke sini. Sensor yang tidak menjawab bukan
        // "tidak ada dinding", dan lewati() memang membatalkan misi untuk itu.
        const int blk = _lidar.getDistance(LIDAR_BACK);
        if (blk == LIDAR_JAUH || blk >= LIDAR_MAX_CM) {
            Serial.print("  sensor belakang JAUH (");
            Serial.print(blk); Serial.println(" cm) -- tidak ada dinding untuk dituju.");
            lewati("mundur dilewati -- tidak ada dinding di belakang.");
            return false;
        }
        if (!_nav.setelBelakangMulai((int)_cm[_i])) {
            lewati("mundur ditolak -- sebabnya tercetak di atas.");
            return false;
        }
        _ruasAwal = _robot.jarakCm();
        _serongT0 = 0;
        return true;
    }

    ModeNav m = (kemudiRuas(_i) == KMD_KIRI) ? NAV_ARENA_KIRI : NAV_ARENA_KANAN;
    _nav.navMulai(m);
    if (_nav.navMode() != m) {
        lewati("gagal memulai navigasi untuk ruas ini -- sebabnya tercetak di atas.");
        return false;
    }

    // DIPASANG LAGI SESUDAH navMulai(), dan ini bukan kehati-hatian berlebih.
    // Ruas beruntun tanpa aksi disambung TANPA menghentikan navigasi, jadi saat
    // ruas berikutnya masuk, navMulai() menemukan mode masih hidup dan
    // memanggil navBerhenti("diambil alih perintah navigasi") -- yang
    // MEMULIHKAN abaikanDepan(false). Bendera buta yang dipasang belasan baris
    // di atas terhapus dua baris kemudian, dan ruas turunan berjalan dengan
    // sensor depan HIDUP: berkasnya menembak lantai miring, terbaca sebagai
    // halangan mendekat, lalu "halangan di depan -- arah arena TIDAK diubah
    // sendiri" menghentikan misi di tengah ruas 3.
    //
    // Gejalanya BERSELANG-SELING, dan itu yang membuatnya lama tak terlihat:
    // kalau badan keluar dari ruas sebelumnya cukup menyerong, ruasMasuk()
    // memilih pivot dulu, navigasi mampir ke NAV_DIAM, navMulai() tidak perlu
    // mengambil alih, dan bendera butanya selamat -- ruas yang sama lolos.
    // Yang di atas TIDAK dihapus: ia yang MEMBERSIHKAN bendera untuk ruas
    // HNT_LANGSUNG/HNT_SISI yang keluar lebih dulu lewat return di atas.
    _nav.abaikanDepan(x.abaikanDepan);

    // Sasaran kemudi fase jalan. Tanpa ini mode arena mengunci ke mata angin
    // TERDEKAT dari yaw sekarang, dan pada ruas menyerong 45 der jarak ke dua
    // mata angin sama besar: pilihannya lemparan koin, dan yang salah menarik
    // badan 90 der dari yang dimaksud sepanjang ruas. Dipasang SESUDAH
    // navMulai() karena navBerhenti() yang dipanggilnya melepas kunci ini,
    // persis seperti yang terjadi pada abaikanDepan di atas.
    _nav.kunciHeading(headingRuas(_i));

    _ruasAwal = _robot.jarakCm();
    _serongT0 = 0;

    // ACUAN PITCH HNT_PUNCAK, dicatat di sini bersama titik nol yang lain.
    // Badan sudah menghadap arah ruas dan profil gaitnya sudah terpasang, jadi
    // kemiringan yang diperintahkan TANJAK_PITCH_DEG sudah ikut terbaca --
    // itulah yang membuat selisih terhadap angka ini bersih dari kemiringan
    // yang kita sendiri minta.
    _pitchAwal    = _nav.imuBicara() ? _nav.pitchKini() : NAN;
    _naikTerlihat = false;
    _datarT0      = 0;

    // Titik nol jarak tempuh, dicatat SEKARANG: badan sudah lurus menghadap
    // arah ruas, jadi berkas sensor belakang tegak lurus dinding START dan
    // angkanya jarak yang sebenarnya. Badan yang masih menyerong membuat
    // berkasnya memanjang 1/cos(sudut) -- titik nol yang salah.
    if (x.henti == HNT_BELAKANG) {
        int b0 = _lidar.getDistance(LIDAR_BACK);
        if (b0 == LIDAR_MATI || b0 == LIDAR_JAUH) {
            lewati("dinding START tidak terbaca sensor belakang -- titik nol tak bisa dicatat.");
            return false;
        }
        _blkAwal = (float)b0;
        Serial.print("  titik nol "); Serial.print(_blkAwal, 1);
        Serial.print(" cm -> berhenti di bacaan "); Serial.print(_blkAwal + _cm[_i], 1);
        Serial.println(" cm");
    }
    return true;
}

// ====================================================================
// PENJAGA SELAMA RUAS BERJALAN
// ====================================================================
bool Misi::ruasSehat() {
    // Ruas GESER punya penjaganya sendiri DI DALAM Navigation (sensor buta,
    // halangan di arah geser, batas waktu), dan tidak satu pun penjaga di
    // bawah berlaku untuknya: ia tidak memakai mode arena, tidak menempuh
    // jarak maju, dan memang tidak menghadap arah ruas selama bergeser.
    if (RUAS[_i].henti == HNT_SISI) {
        if (_nav.ratakanSedangJalan()) return true;
        if (!_nav.ratakanTercapai()) {
            lewati("perataan sisi berhenti sebelum sasaran -- sebabnya tercetak di atas.");
            return false;
        }
        return true;
    }

    // Ruas MUNDUR: alasannya sama persis dengan GESER di atas. Penjaganya ada
    // di dalam NAV_SETEL_BLK (sensor belakang mati di tengah jalan, halangan,
    // MUNDUR_BATAS_MS), dan tidak satu pun penjaga mode arena di bawah
    // berlaku -- robot memang tidak menghadap arah jalannya saat mundur.
    if (RUAS[_i].henti == HNT_MUNDUR) {
        if (_nav.setelBelakangSedangJalan()) return true;
        if (!_nav.setelBelakangTercapai()) {
            lewati("mundur berhenti sebelum sasaran -- sebabnya tercetak di atas.");
            return false;
        }
        return true;
    }

    // 1) Navigasi masih milik kita? Satu pemeriksaan ini menangkap SEMUANYA:
    //    navigasi yang berhenti sendiri (sensor depan mati, terjebak, dinding
    //    hilang terlalu lama) DAN navigasi yang diambil alih dari serial
    //    ('f', 'F', 'p', 'o', 'C', ...). Alternatifnya menaruh sebelas kait
    //    di parser, dan yang kedua belas pasti terlupa.
    // kemudiRuas(), BUKAN RUAS[_i].kemudi. Di mode cermin keduanya berbeda,
    // dan ruasJalan() menyalakan navigasi memakai yang tercermin -- kalau
    // pemeriksa ini memakai kolom mentah, ia mengira navigasi 'diambil alih'
    // pada tiap ruas ikut-dinding dan membatalkan misi di langkah pertama.
    ModeNav m = (kemudiRuas(_i) == KMD_KIRI) ? NAV_ARENA_KIRI : NAV_ARENA_KANAN;
    if (_nav.navMode() != m) {
        lewati("navigasi berhenti atau diambil alih -- sebabnya tercetak di atas.");
        return false;
    }

    if (lewat() > MISI_RUAS_BATAS_MS) {
        _nav.navBerhenti("batas waktu ruas.");
        lewati("ruas tidak selesai dalam batas waktunya.");
        return false;
    }

    // Pagar jarak -- lihat MISI_RUAS_MAKS_CM. Berjalan lebih jauh dari ini
    // berarti syarat hentinya tidak akan pernah terpenuhi, apa pun sebabnya.
    const float tempuhRuas = _robot.jarakCm() - _ruasAwal;
    if (tempuhRuas > MISI_RUAS_MAKS_CM) {
        _nav.navBerhenti("pagar jarak ruas.");
        Serial.print("  sudah menempuh "); Serial.print(tempuhRuas, 1);
        Serial.print(" cm, pagar "); Serial.print(MISI_RUAS_MAKS_CM, 0);
        Serial.println(" cm.");
        lewati("ruas berjalan jauh melampaui ukuran arena -- syarat hentinya tidak tercapai.");
        return false;
    }

    // 2) Navigasi memilih mata angin LAIN. Itu 90 der sekaligus, bukan
    //    goyangan, dan tiap langkah sesudahnya dihitung ke arah yang salah.
    //    Tidak ada gunanya menunggu.
    if (_nav.arahDituju() != (int8_t)_arah[_i]) {
        _nav.navBerhenti("navigasi berbelok ke mata angin lain di tengah ruas.");
        lewati("navigasi berpindah arah -- sisa ruas akan diukur ke arah yang salah.");
        return false;
    }

    // 3) Badan terserong sementara sambil tetap MENUJU arah yang sama. Di
    //    lantai pecah itu normal: kaki tergelincir, mode arena menariknya
    //    kembali selewat beberapa langkah. Yang ditangkap hanya serong BESAR
    //    yang BERTAHAN -- misalnya robot menyangkut sehingga koreksinya tidak
    //    pernah menang.
    float simpang = _nav.simpangHeading(headingRuas(_i));
    if (isnan(simpang)) {
        _nav.navBerhenti("IMU tidak memberi data di tengah ruas.");
        lewati("acuan heading hilang -- jarak tempuh tidak bisa dipercaya.");
        return false;
    }
    if (fabsf(simpang) <= MISI_SERONG_DEG) {
        _serongT0 = 0;
    } else {
        uint32_t batas = (uint32_t)(MISI_SERONG_SIKLUS * _robot.gaitProfile().cycleTime);
        if (batas < MISI_SERONG_MIN_MS) batas = MISI_SERONG_MIN_MS;
        if (_serongT0 == 0) _serongT0 = millis();
        else if (millis() - _serongT0 > batas) {
            _nav.navBerhenti("heading menyimpang di tengah ruas.");
            Serial.print("  serong "); Serial.print(simpang, 1);
            Serial.print(" der bertahan lebih dari "); Serial.print(batas);
            Serial.println(" ms.");
            lewati("robot tidak lagi menghadap arah ruas -- jarak tempuh tidak bisa dipercaya.");
            return false;
        }
    }
    return true;
}

// ====================================================================
// SYARAT HENTI
// ====================================================================
bool Misi::ruasSelesai() {
    const Ruas& x = RUAS[_i];

    if (x.henti == HNT_LANGSUNG) return true;
    if (x.henti == HNT_ODO)      return (_robot.jarakCm() - _ruasAwal) >= _cm[_i];

    // Navigation sudah berhenti sendiri begitu sasaran tercapai; kalau ia
    // berhenti karena GAGAL, ruasSehat() sudah membatalkan misi lebih dulu.
    if (x.henti == HNT_SISI)     return !_nav.ratakanSedangJalan();
    if (x.henti == HNT_MUNDUR)   return !_nav.setelBelakangSedangJalan();

    if (x.henti == HNT_PUNCAK) {
        // DINDING DEPAN CUMA BERLAKU KALAU `nilai` > 0. Nol = dimatikan,
        // gyro sendirian.
        //
        // Dimatikan di R-9 pada 18 Sep 2026 sesudah dicoba: di tanjakan berkas
        // depan sering mengenai MUKA ANAK TANGGA, bukan dinding seberang, dan
        // ruasnya berakhir di tengah pendakian. Gerbang mendaki tidak
        // menolongnya -- begitu robot benar-benar mendaki, gerbang terbuka dan
        // anak tangga berikutnya langsung memicu.
        //
        // Dibiarkan sebagai pilihan, bukan dibuang: di tanjakan yang lebih
        // landai berkasnya tidak mengenai anak tangga, dan di sana dinding
        // seberang penjaring yang berguna kalau gyro meleset.
        const int dP = _lidar.getDistance(LIDAR_FRONT);
        // MATI dan JAUH bukan "sudah sampai" -- yang satu sensor putus, yang
        // lain lorong masih terbuka.
        const bool dindingDekat = (_cm[_i] > 0.0f &&
                                   dP != LIDAR_MATI && dP != LIDAR_JAUH &&
                                   (float)dP <= _cm[_i]);

        // IMU BISU: gerbang mendaki tidak bisa dipakai, jadi tinggal dinding.
        // Menunggu gyro yang tidak pernah datang berarti berjalan sampai batas
        // waktu ruas, di tangga.
        if (isnan(_pitchAwal) || !_nav.imuBicara()) {
            if (dindingDekat) {
                Serial.print("  PUNCAK: IMU bisu, berhenti dari dinding depan ");
                Serial.print(dP); Serial.println(" cm.");
            }
            return dindingDekat;
        }

        const float simpang = fabsf(_nav.pitchKini() - _pitchAwal);

        // GERBANG MENDAKI. Selama robot belum pernah benar-benar miring, ruas
        // ini TIDAK BOLEH berakhir -- di kaki tangga badan memang datar, dan
        // berkas depan yang menyentuh muka anak tangga pertama akan
        // mengakhirinya juga.
        if (!_naikTerlihat) {
            if (simpang >= PUNCAK_NAIK_DEG) {
                _naikTerlihat = true;
                Serial.print("  PUNCAK: mendaki terlihat, simpang ");
                Serial.print(simpang, 1); Serial.println(" der.");
            }
            return false;
        }

        if (dindingDekat) {
            Serial.print("  PUNCAK: dinding depan "); Serial.print(dP);
            Serial.print(" cm <= "); Serial.print(_cm[_i], 0);
            Serial.println(" cm -- ruas selesai.");
            return true;
        }

        // DATAR LAGI, dan harus BERTAHAN. Satu sampel datar muncul juga di
        // tengah tanjakan tiap kali kaki depan menapak anak tangga berikutnya.
        if (simpang > PUNCAK_DATAR_DEG) { _datarT0 = 0; return false; }
        if (_datarT0 == 0) { _datarT0 = millis(); return false; }
        if (millis() - _datarT0 < PUNCAK_DATAR_MS) return false;

        Serial.print("  PUNCAK: datar lagi ("); Serial.print(simpang, 1);
        Serial.print(" der) selama "); Serial.print(PUNCAK_DATAR_MS);
        Serial.println(" ms -- ruas selesai.");
        return true;
    }

    uint8_t kanal = (x.henti == HNT_DEPAN) ? LIDAR_FRONT : LIDAR_BACK;
    int     d     = _lidar.getDistance(kanal);

    // Tiga keadaan, tiga perlakuan. Inilah yang tidak bisa dilakukan FSM
    // generasi lama: di sana ketiganya sama-sama -1.
    //   MATI  : jangan hitung apa pun. Sensor putus tidak boleh terhitung
    //           sebagai "sudah sampai". Navigasi yang berhak menghentikan,
    //           dan ruasSehat() menangkapnya iterasi berikutnya.
    //   JAUH  : untuk sensor DEPAN berarti lorong masih terbuka -- bukan
    //           "sangat dekat". Untuk sensor BELAKANG berarti dinding START
    //           hilang dari pandangan; memicu dari situ berarti berhenti di
    //           tempat acak, jadi biar batas waktu yang menghentikannya.
    if (d == LIDAR_MATI || d == LIDAR_JAUH) { _n = 0; return false; }

    // Hitung HANYA saat ada sampel BARU. update() dipanggil ribuan kali per
    // detik, sedangkan tiap sensor cuma menghasilkan satu pengukuran tiap
    // ~150 ms (round-robin enam channel). Tanpa gerbang ini, "3 sampel
    // berturut-turut" berarti tiga iterasi loop yang membaca angka yang sama.
    uint32_t st = _lidar.stempelSampel(kanal);
    if (st == _stempel) return false;
    _stempel = st;

    bool kena = (x.henti == HNT_DEPAN) ? ((float)d <= _cm[_i])
                                       : ((float)d >= _blkAwal + _cm[_i]);
    if (!kena) { _n = 0; return false; }
    return (++_n >= MISI_SAMPEL_N);
}

// ====================================================================

bool Misi::lidarMati() {
    // NUM_LIDAR, bukan cuma kanal yang dipakai ruas ini. Sensor yang berhenti
    // menjawab hampir tidak pernah sendirian -- mux, catu, atau kabel bersama.
    // Ruas berikutnya yang memakainya akan menemuinya juga, dan menemukannya
    // di tengah tanjakan lebih mahal daripada berhenti sekarang.
    return _lidar.jumlahHidup() < NUM_LIDAR;
}

bool Misi::pulihkanLidar() {
    if (_lidarUlang >= LIDAR_ULANG_MAKS) {
        Serial.print("  sudah "); Serial.print(_lidarUlang);
        Serial.println(" kali dipindai ulang dan tetap putus -- menyerah.");
        return false;
    }
    _lidarUlang++;

    // BERHENTI DULU. pindaiI2C() memblokir beberapa ratus milidetik sambil
    // mematikan dan menghidupkan tiap kanal mux. Robot yang masih melangkah
    // selama itu berjalan buta, dan yang menghentikannya cuma ruas berikutnya
    // -- terlambat.
    _nav.navBerhenti("LiDAR dipindai ulang.");

    Serial.print("\n!! LiDAR PUTUS di tengah misi -- pindai ulang (");
    Serial.print(_lidarUlang); Serial.print(" dari ");
    Serial.print(LIDAR_ULANG_MAKS); Serial.println(")");

    _lidar.pindaiI2C();

    const bool pulih = !lidarMati();
    Serial.println(pulih ? "  PULIH."
                         : "  masih ada yang tidak menjawab.");
    return pulih;
}

void Misi::lewati(const char* sebab) {
    // LiDAR MATI MENANG atas 'lewati'. Sebab lunak apa pun yang terjadi
    // bersamaan dengan sensor yang berhenti menjawab bukan lagi sebab lunak:
    // ruas berikutnya akan gagal dengan cara yang sama, dan meneruskan cuma
    // memindahkan kegagalan ke tempat yang lebih sulit dibaca.
    if (lidarMati()) {
        // Robot tidak boleh disentuh saat lomba, jadi ia mencoba sendiri dulu.
        // Berhasil: ruas ini DIULANG dari awal, bukan dilewati -- ia berhenti
        // karena sensornya berkedip, bukan karena ruasnya tidak bisa dikerjakan.
        if (pulihkanLidar()) {
            Serial.println("  CATATAN: odometri ruas ini dihitung dari nol lagi,");
            Serial.println("  jadi jarak yang sudah ditempuh akan ditempuh dua kali.");
            ruasMasuk();
            return;
        }
        gagal("LiDAR tidak merespons -- periksa 'l' lalu 'I'.");
        return;
    }
    // Mode UKUR ('m7') dipakai operator untuk membaca satu ruas, bukan untuk
    // menyelesaikan lintasan. Maju diam-diam ke ruas berikutnya di sana
    // menghilangkan justru yang sedang diamati.
    if (_stat == MISI_UKUR) { gagal(sebab); return; }
    Serial.print("\n== RUAS "); Serial.print(_i);
    Serial.print(" DILEWATI ("); Serial.print(RUAS[_i].nama); Serial.println(")");
    Serial.print("   sebab: "); Serial.println(sebab);
    Serial.println("   Misi DITERUSKAN ke ruas berikutnya. Ruas ini TIDAK berpoin.");

    _nav.navBerhenti("ruas dilewati.");
    _nav.abaikanDepan(false);
    _nav.setTengah(false);
    ruasBerikut(false);
}

void Misi::ruasBerikut(bool berpoin) {
    // POIN DICATAT DI SINI, dan cuma di sini. Ini satu-satunya tempat sebuah
    // ruas dinyatakan SELESAI. Ruas yang gagal atau dibatalkan tidak pernah
    // lewat sini; ruas yang DILEWATI lewat sini dengan berpoin=false.
    if (berpoin) gSkor.ruasSelesai(_i);

    _i++;
    if (_i >= RUAS_N || _i > _iAkhir) {
        _nav.navBerhenti("lintasan selesai.");
        _nav.abaikanDepan(false);
        _nav.setTengah(false);
        _robot.profileFlat();
        masuk(MISI_SELESAI);
        Serial.println(_iAkhir >= RUAS_N - 1
                       ? "\n=== FINISH -- seluruh lintasan selesai ==="
                       : "\n=== BERHENTI di ruas akhir yang diminta ===");
        Serial.print("  waktu total: "); Serial.print(lewat() / 1000);
        Serial.println(" detik.");
        return;
    }
    ruasMasuk();
}

void Misi::update() {
    if (!berjalan()) return;

    switch (_stat) {

    case MISI_PIVOT: {
        // Navigation punya timeout PIVOT_BATAS_MS sendiri, jadi tinggal
        // menunggu ia berhenti lalu MEMERIKSA hasilnya: pivot yang selesai
        // dan pivot yang timeout/dibatalkan sama-sama berakhir di NAV_DIAM,
        // jadi heading akhir yang membedakannya.
        if (_nav.pivotSedangJalan()) return;
        const float sasar = headingRuas(_i);
        if (!_nav.diHeading(sasar)) {
            // SEKALI ULANG sebelum menyerah. Percobaan kedua berangkat dari
            // heading yang sudah lebih dekat dan mendapat jatah
            // PIVOT_BATAS_MS yang baru, jadi pivot yang cuma KEHABISAN WAKTU
            // -- kaki terganjal, lantai licin, badan mentok dinding sebentar
            // -- sering selesai di percobaan itu. Yang TIDAK ditolong:
            // kompas belum tercatat, IMU lepas, atau c0..c3 yang memang
            // salah; ketiganya meleset dengan simpangan yang sama besar dua
            // kali, dan angka yang tercetak di bawah yang membedakannya.
            float simpang = _nav.simpangHeading(sasar);
            if (_pivotUlang == 0) {
                _pivotUlang = 1;
                Serial.print("  pivot meleset "); Serial.print(simpang, 1);
                Serial.println(" der -- MENGULANG sekali.");
                _nav.pivotKe(sasar);
                if (!_nav.pivotSedangJalan()) {
                    lewati("pivot ulangan tidak mau jalan -- sebabnya tercetak di atas.");
                    return;
                }
                masuk(MISI_PIVOT);
                return;
            }
            Serial.print("  ulangan pun meleset "); Serial.print(simpang, 1);
            Serial.println(" der.");
            lewati("pivot tidak sampai walau sudah diulang -- periksa 'k' (kompas c0..c3) dan IMU.");
            return;
        }
        // Badan baru saja berputar. Sampel LiDAR yang ada sekarang lahir dari
        // heading lama, jadi ruas yang berhenti pada sensor harus menunggu
        // berkasnya berganti isi dulu -- lihat pasangProfil().
        _pivotBaru = true;
        ruasBerangkat();
        break;
    }

    case MISI_SETEL: {
        // Navigasi sudah diam (pasangProfil() yang menghentikannya), jadi
        // penjaga servo milik Navigation tidak berlaku -- jaga sendiri, sama
        // seperti MISI_LENGAN dan MISI_KONFIRM.
        if (!_robot.isArmed()) { gagal("servo dilemaskan saat menunggu profil gait."); return; }

        if (!_robot.gaitProfilTenang()) {
            if (lewat() > MISI_SETEL_BATAS_MS) {
                gagal("ramp profil gait tidak kunjung selesai -- gait tidak di-update?");
                return;
            }
            _tenangT0 = 0;      // sempat tenang lalu bergerak lagi -> hitung dari nol
            return;
        }
        if (_tenangT0 == 0) _tenangT0 = millis();
        if (millis() - _tenangT0 < MISI_LIDAR_SEGAR_MS) return;

        ruasBerangkat();
        break;
    }

    case MISI_JALAN: {
        if (RUAS[_i].henti != HNT_LANGSUNG && !ruasSehat()) return;
        if (!ruasSelesai()) return;

        const Ruas& x = RUAS[_i];

        // Berhenti HANYA kalau ada yang harus dikerjakan di tempat. Ruas
        // beruntun yang searah disambung tanpa berhenti: menghentikan lalu
        // menjalankan lagi tiap ganti profil membuat robot tersendat di
        // perbatasan, dan perbatasan itulah bibir rintangan.
        if (x.aksi == AKS_TIDAK_ADA) { ruasBerikut(); return; }

        _nav.navBerhenti("ujung ruas tercapai.");
        _nav.abaikanDepan(false);

        if (x.aksi == AKS_KONFIRM) {
            masuk(MISI_KONFIRM);
            Serial.println("\n=== BERHENTI -- MENUNGGU KEPUTUSAN OPERATOR ===");
            Serial.println("  'm2' = lanjut  |  'm3' = ulangi ruas ini  |  'm0' = batal");
            return;
        }

        // Pemicu ke Raspi 5: robot sudah diam di depan korban, silakan
        // mulai deteksi. Tempatnya di sini dan bukan di MISI_LENGAN karena
        // cabang ini cuma dilewati SEKALI per ruas -- MISI_LENGAN dipanggil
        // tiap loop. Kirim saja, tidak menunggu jawaban: TAHAP 1 memang tidak
        // memakai balasannya -- pelurusan kiri-kanan dikerjakan kamera SEBELUM
        // ruas ini, dan jarak datang dari LiDAR depan lewat HNT_DEPAN.
        KORBAN_SERIAL.print("#KORBAN ");
        KORBAN_SERIAL.print(x.aksi == AKS_AMBIL ? "AMBIL " : "TARUH ");
        KORBAN_SERIAL.print(_i);
        // ARAH MATA ANGIN RUAS INI (0..3) DAN SAKLAR CERMIN, dua kolom
        // tambahan sejak 18 Sep 2026.
        //
        // Fase CARI di Raspi memakai keduanya: 'o<arah>' untuk meluruskan
        // badan ke kompas sebelum menengahkan, dan cermin untuk memilih ke
        // mana ia menggeser mencari korban. Dikirim dari sini, bukan dihitung
        // ulang di Raspi, karena _arah[] SUDAH hasil pencerminan -- aturan
        // cermin yang disalin ke sisi lain adalah aturan yang akan menyimpang
        // diam-diam begitu salah satunya disentuh.
        //
        // Kolom TAMBAHAN di belakang, bukan format baru: pembaca lama yang
        // hanya mengambil dua kolom pertama tetap bekerja apa adanya.
        KORBAN_SERIAL.print(' '); KORBAN_SERIAL.print(_arah[_i]);
        KORBAN_SERIAL.print(' '); KORBAN_SERIAL.println(arenaCermin() ? 1 : 0);
        _visiJalan = true;      // ditutup '#LEPAS', termasuk lewat lepasVisi()

        // PARKIR, bukan langsung menurunkan capit. Ini yang dikembalikan
        // 15 Sep 2026 sesudah v1.15 membuangnya: tanpa parkir, Raspi melihat
        // korban tapi tidak boleh menggerakkan badan -- dua penguasa untuk
        // satu robot -- jadi ia hanya menonton sampai nama ruasnya berganti.
        // Dari luar: robot melihat korban lalu pergi.
        //
        // Hanya untuk AKS_AMBIL: menaruh tidak butuh kamera, titik lepasnya
        // ditentukan tabel.
        if (_tungguVision && x.aksi == AKS_AMBIL) {
            _parkirVision = true;
            _yawUjung = _nav.yawKini();
            masuk(MISI_KONFIRM);
            Serial.println("\n=== PARKIR UNTUK VISION -- menunggu 'm2' dari Raspi ===");
            Serial.println("  Kendali kaki MILIK RASPI sampai 'm2'. Ia menengahkan");
            Serial.println("  badan ke korban, menahannya, lalu menjawab.");
            Serial.println("  'm2' = lanjut ke sekuens capit  |  'm3' = ulangi ruas");
            Serial.println("  'm0' = batal  |  'm8 0' = matikan mode tunggu ini");
            return;
        }

        // Titik nol pengukuran serong. Diambil SESUDAH navBerhenti(), jadi
        // yang terukur sesudahnya murni apa yang terjadi saat gait berhenti
        // dan saat lengan bergerak -- bukan kemudi ruas yang baru saja usai.
        _yawUjung = _nav.yawKini();

        masuk(MISI_LENGAN);
        Serial.print("\n=== "); Serial.print(x.aksi == AKS_AMBIL ? "ANGKAT" : "TARUH");
        Serial.print(" KORBAN -- ruas "); Serial.print(_i); Serial.println(" ===");
        if (LENGAN_KORBAN_AKTIF)
            Serial.printf("  pose sendi tetap: bahu %.0f, siku %.0f, pergelangan %.0f der\n",
                          (double)(x.aksi == AKS_AMBIL ? KORBAN_SIAP_BAHU : KORBAN_LEPAS_BAHU),
                          (double)(x.aksi == AKS_AMBIL ? KORBAN_SIAP_SIKU : KORBAN_LEPAS_SIKU),
                          (double)(x.aksi == AKS_AMBIL ? KORBAN_SIAP_PRG  : KORBAN_LEPAS_PRG));
        else
            Serial.println("  DILEWATI -- LENGAN_KORBAN_AKTIF 0 di config.h");
        break;
    }

    case MISI_LENGAN: {
        // Navigasi sudah diam di sini (kita yang menghentikannya), jadi
        // penjaga servo milik Navigation tidak lagi berlaku. Jaga sendiri.
        if (!_robot.isArmed()) { gagal("servo dilemaskan saat sekuens lengan."); return; }

        // Uji operator: pose yang sama, tapi tidak ada ruas yang menunggu di
        // ujungnya dan tidak ada capit yang perlu dicatat isinya.
        if (_uji) {
            const bool selesai = _ujiAmbil
                                     // mundurMm 0: uji operator dijalankan di
                                     // meja tanpa korban, dan menarik badan
                                     // mundur di sana cuma menyembunyikan
                                     // bentuk busur turun yang mau dilihat.
                                     ? sekuensAmbil(_robot, ARM_DEPAN, _langkah, _t0,
                                                    false, 0.0f, 0.0f, 0.0f)
                                     : sekuensTaruh(_robot, ARM_DEPAN, _langkah, _t0);
            if (!selesai) return;
            _uji  = false;
            _stat = MISI_DIAM;
            Serial.println("Sekuens lengan selesai.");
            return;
        }

        // Sebelum satu servo lengan pun bergerak: berapa derajat badan sudah
        // berpindah sejak ruas berhenti? Itu sumbangan GAIT, bukan lengan.
        if (_langkah == 0 && !isnan(_yawUjung))
            Serial.printf("  serong sejak ruas berhenti, SEBELUM lengan: %+.1f der\n",
                          (double)selisihYaw());

        const Ruas& x = RUAS[_i];
        // Saklar config.h. Hubung-singkat, jadi dengan saklar mati sekuensnya
        // tidak pernah dipanggil -- tapi tetap DIRUJUK, sehingga kompilator
        // tidak mengeluh soal fungsi tak terpakai dan isi sekuens tetap
        // ikut diperiksa saat kompilasi.
        const bool usai = !LENGAN_KORBAN_AKTIF ||
                          ((x.aksi == AKS_AMBIL)
                               ? sekuensAmbil(_robot, x.aksiA, _langkah, _t0,
                                              x.condong, koreksiYawRuas(),
                                              x.mundurMm, x.condongMm)
                               : sekuensTaruh(_robot, x.aksiA, _langkah, _t0));
        if (!usai) return;

        // Tidak diperbarui saat dilewati: capit benar-benar kosong, dan
        // ringkasan 'm1' tidak boleh berbohong soal itu.
        if (!isnan(_yawUjung)) {
            Serial.printf("  serong sejak ruas berhenti, SESUDAH lengan: %+.1f der\n",
                          (double)selisihYaw());
            _yawUjung = NAN;
        }

        if (LENGAN_KORBAN_AKTIF) _korban[x.aksiA & 1] = (x.aksi == AKS_AMBIL);

        // SERAH-TERIMA BALIK. Capit selesai, tapi Raspi bisa MASIH memegang
        // kaki: ia menahan pose badan yang dipakai menengahkan tadi.
        // Melangkah sekarang berarti Teensy meng-override penguasa yang belum
        // melepas, dan pose yang masih berlaku menyentak badan di langkah
        // pertama. Jadi: beri tahu, lalu TUNGGU jawabannya.
        //
        // Hanya untuk ruas yang tadi memang diparkir ke vision. Ruas TARUH
        // tidak pernah menyerahkan kaki.
        if (_parkirVision) {
            _parkirVision = false;
            _visiJalan    = false;
            KORBAN_SERIAL.print("#LEPAS ");
            KORBAN_SERIAL.println(_i);
            masuk(MISI_LEPAS);
            Serial.println("\n=== CAPIT SELESAI -- menunggu 'm9' dari Raspi ===");
            Serial.println("  'm9' = Raspi sudah tidak memegang kaki, misi boleh lanjut.");
            return;
        }

        ruasBerikut();
        break;
    }

    case MISI_LEPAS: {
        if (!_robot.isArmed()) { gagal("servo dilemaskan saat menunggu 'm9'."); return; }
        if (lewat() <= MISI_LEPAS_BATAS_MS) return;   // masih menunggu Raspi

        // Habis waktu TIDAK menggagalkan misi. Raspi yang mati atau kabel
        // serial yang lepas tidak boleh membuat robot berdiri diam sampai jam
        // kontes habis sambil menggenggam korban yang sudah berhasil diangkat.
        Serial.print("!! 'm9' tidak datang dalam ");
        Serial.print(MISI_LEPAS_BATAS_MS / 1000); Serial.println(" detik.");
        Serial.println("   Misi DILANJUTKAN. Kalau Raspi ternyata masih menahan pose");
        Serial.println("   badan, langkah pertama akan menyentak -- periksa HUD.");
        ruasBerikut();
        break;
    }

    case MISI_UKUR:
        // Penjaganya tetap berlaku -- arah, serong, navigasi diambil alih.
        // Yang TIDAK ada cuma syarat hentinya; itu tugas operator.
        ruasSehat();
        break;

    case MISI_KONFIRM:
        if (!_robot.isArmed()) { gagal("servo dilemaskan saat menunggu konfirmasi."); return; }

        // Batas waktu HANYA untuk parkir vision. Ruas AKS_KONFIRM biasa
        // menunggu keputusan MANUSIA, dan manusia tidak boleh di-timeout --
        // ia mungkin sedang mengukur sesuatu.
        if (_parkirVision && lewat() > MISI_VISI_BATAS_MS) {
            _parkirVision = false;
            Serial.print("!! Raspi tidak menjawab dalam ");
            Serial.print(MISI_VISI_BATAS_MS / 1000); Serial.println(" detik.");
            Serial.println("   Sekuens capit DIJALANKAN memakai sudut tetap -- sama");
            Serial.println("   dengan perilaku tanpa vision. Korban tidak hilang karena ini.");
            masuk(MISI_LENGAN);
        }
        break;

    default:
        break;
    }

    // Jam kontes, diperiksa paling akhir supaya ruas yang baru saja selesai
    // tetap tercatat sebelum misinya ditutup. _t0 tidak bisa dipakai untuk
    // ini -- ia di-reset tiap masuk state.
    if (berjalan() && millis() - _tMisi > MISI_TOTAL_BATAS_MS) {
        _nav.navBerhenti("batas waktu kontes 5 menit.");
        gagal("waktu kontes habis (300 detik).");
    }
}

void Misi::jawab(bool lanjut) {
    if (_stat != MISI_KONFIRM) {
        Serial.println("Tidak sedang menunggu konfirmasi. 'm' untuk status.");
        return;
    }
    // DUA UJUNG, dan membedakannya WAJIB. Parkir vision harus lanjut ke
    // sekuens CAPIT; ruas AKS_KONFIRM biasa harus lanjut ke ruas BERIKUTNYA.
    // Menyamakan keduanya berarti 'm2' melompati pengambilannya dan robot
    // berjalan ke ruas berikutnya dengan capit kosong, tanpa satu pun pesan.
    if (_parkirVision) {
        if (lanjut) {
            Serial.println("Dicatat: LANJUT -- kendali kaki kembali ke Teensy.");
            masuk(MISI_LENGAN);       // _parkirVision tetap: dipakai MISI_LENGAN
            return;                   // untuk tahu ia harus mengirim '#LEPAS'
        }
        _parkirVision = false;
        Serial.println("Dicatat: ULANGI ruas ini.");
        ruasMasuk();
        return;
    }

    if (lanjut) { Serial.println("Dicatat: LANJUT."); ruasBerikut(); return; }
    Serial.println("Dicatat: ULANGI ruas ini.");
    ruasMasuk();
}

void Misi::lepasKendali() {
    if (_stat != MISI_LEPAS) {
        Serial.println("Tidak sedang menunggu 'm9'. 'm' untuk status.");
        return;
    }
    Serial.println("Dicatat: Raspi melepas kaki. Misi dilanjutkan.");
    ruasBerikut();
}

void Misi::setTungguVision(int mode) {
    _tungguVision = (mode < 0) ? !_tungguVision : (mode != 0);
    Serial.print("Tunggu vision: ");
    Serial.println(_tungguVision ? "NYALA -- ruas AMBIL parkir menunggu 'm2' dari Raspi"
                                 : "MATI -- capit turun tanpa menunggu kamera");
    if (!_tungguVision && _parkirVision && _stat == MISI_KONFIRM) {
        // Dimatikan SELAGI parkir. Melanjutkan parkir yang sudah tidak diakui
        // siapa pun berarti robot berdiri diam sampai batas waktunya habis.
        _parkirVision = false;
        Serial.println("  Sedang parkir -- dilepas sekarang, capit turun.");
        masuk(MISI_LENGAN);
    }
}

void Misi::setRuasCm(uint8_t idx, float cm) {
    if (idx >= RUAS_N) {
        Serial.print("Ruas "); Serial.print(idx); Serial.print(" tidak ada -- hanya 0..");
        Serial.println(RUAS_N - 1);
        return;
    }
    if (RUAS[idx].henti == HNT_LANGSUNG) {
        Serial.print("Ruas "); Serial.print(idx);
        Serial.println(" tidak berjalan (hanya aksi) -- tidak ada panjang untuk disetel.");
        return;
    }

    // Batas atas & bawah menurut CARA ruas itu berhenti, bukan satu rentang
    // untuk semuanya: ambang sensor punya batas fisik yang berbeda dari
    // panjang odometri.
    float lo = 5.0f, hi = 400.0f;
    if (RUAS[idx].henti == HNT_DEPAN)    { lo = (float)FRONT_STOP_CM + 1.0f; hi = 200.0f; }
    // PUNCAK memakai sensor DEPAN, jadi batasnya sama persis dengan HNT_DEPAN.
    if (RUAS[idx].henti == HNT_PUNCAK)   { lo = (float)FRONT_STOP_CM + 1.0f; hi = 200.0f; }
    if (RUAS[idx].henti == HNT_BELAKANG) { hi = (float)LIDAR_MAX_CM - 15.0f; }
    if (RUAS[idx].henti == HNT_SISI)     { lo = WALL_MIN_CM + 1.0f; hi = (float)LIDAR_MAX_CM; }
    // MUNDUR: batas bawahnya sama dengan yang dipakai setelBelakangMulai()
    // untuk menolak sasaran -- kalau berbeda, 'm7' akan menerima angka yang
    // nanti ditolak lagi di arena, sesudah robot terlanjur berangkat.
    if (RUAS[idx].henti == HNT_MUNDUR)   { lo = WALL_MIN_CM + 1.0f; hi = (float)LIDAR_MAX_CM; }

    float v = clampf(cm, lo, hi);
    Serial.print("ruas "); Serial.print(idx); Serial.print(" ("); Serial.print(RUAS[idx].nama);
    Serial.print("): ");
    if (_cm[idx] < 0.0f) Serial.print("belum diukur"); else Serial.print(_cm[idx], 1);
    Serial.print(" -> "); Serial.print(v, 1); Serial.println(" cm");
    if (fabsf(v - cm) > 1e-3f) {
        Serial.print("  (diminta "); Serial.print(cm, 1); Serial.print(", DI-CLAMP ke ");
        Serial.print(lo, 0); Serial.print(" .. "); Serial.print(hi, 0); Serial.println(")");
    }
    _cm[idx] = v;
    Serial.println("  Hanya di RAM -- belum ada slot EEPROM untuk parameter misi.");
}

void Misi::cetakRuas(uint8_t idx) {
    const Ruas& x = RUAS[idx];
    Serial.print(idx == _i && berjalan() ? " >" : "  ");
    if (idx < 10) Serial.print(' ');
    Serial.print(idx); Serial.print("  ");
    Serial.print(_nav.namaArah(_arah[idx]));
    for (uint8_t k = strlen(_nav.namaArah(_arah[idx])); k < 8; k++) Serial.print(' ');
    // YANG DICETAK ADALAH YANG BERLAKU, bukan yang tertulis di tabel. Di mode
    // cermin keduanya berbeda, dan tabel yang menampilkan sumber sementara
    // robot menjalankan cerminnya adalah tabel yang menyesatkan justru saat
    // paling dibutuhkan.
    switch (belokRuas(idx)) {
        case BLK_KANAN: Serial.print("kanan"); break;
        case BLK_KIRI:  Serial.print("kiri "); break;
        case BLK_BALIK: Serial.print("balik"); break;
        default:        Serial.print("lurus"); break;
    }
    // Serong dicetak MENEMPEL pada beloknya, bukan di kolom sendiri: ia
    // memang bagian dari belok itu, cuma satuannya derajat.
    const float ptr = putarRuas(idx);
    if (fabsf(ptr) > 0.5f)     { Serial.print(ptr > 0 ? '+' : '-');
                                 Serial.print(fabsf(ptr), 0); }
    else                       { Serial.print("  "); }
    Serial.print(' ');

    switch (kemudiRuas(idx)) {
        case KMD_KIRI:   Serial.print("kiri   "); break;
        case KMD_TENGAH: Serial.print("tengah "); break;
        default:         Serial.print("kanan  "); break;
    }
    switch (x.profil) {
        case PRF_TANGGA:   Serial.print("TANGGA   "); break;
        case PRF_MERUNDUK: Serial.print("MERUNDUK "); break;
        case PRF_SEMPIT:   Serial.print("SEMPIT   "); break;
        case PRF_KAIL:     Serial.print("KAIL     "); break;
        case PRF_TANJAK:   Serial.print("TANJAK   "); break;
        default:           Serial.print("datar    "); break;
    }
    switch (x.henti) {
        case HNT_ODO:       Serial.print("odo  "); break;
        case HNT_DEPAN:     Serial.print("dpn  "); break;
        case HNT_BELAKANG:  Serial.print("blk  "); break;
        case HNT_SISI:      Serial.print("sisi "); break;
        case HNT_MUNDUR:    Serial.print("mdr  "); break;
        case HNT_PUNCAK:    Serial.print("pck  "); break;
        default:            Serial.print("--   "); break;
    }
    if (x.henti == HNT_LANGSUNG)   Serial.print("     ");
    else if (_cm[idx] < 0.0f)      Serial.print("  ?  ");
    else { if (_cm[idx] < 100.0f) Serial.print(' '); Serial.print(_cm[idx], 0); Serial.print(" "); }

    Serial.print(x.abaikanDepan ? " buta " : "      ");
    if      (x.aksi == AKS_AMBIL)   Serial.print("ANGKAT  ");
    else if (x.aksi == AKS_TARUH)   Serial.print("TARUH   ");
    else if (x.aksi == AKS_KONFIRM) Serial.print("konfirm ");
    else                            Serial.print("        ");
    Serial.println(x.nama);
}

void Misi::tabel() {
    Serial.println("\n--- TABEL LINTASAN ---");
    // MODE CERMIN DIUMUMKAN DI KEPALA TABEL. Tanpa baris ini, satu-satunya
    // tanda bahwa robot menjalankan cerminnya adalah kolom belok/kemudi yang
    // "terasa terbalik" -- dan itu baru disadari sesudah robot berjalan ke
    // arah yang salah.
    if (arenaCermin()) {
        Serial.println("  ARENA CERMIN AKTIF -- kiri dan kanan DITUKAR.");
        Serial.println("  Yang tercetak di bawah sudah hasil pencerminan, bukan isi tabel.");
        Serial.println("  Matikan dengan 'Qarena.mirror 0' atau tombol D3.");
    }
    Serial.println("  #  arah    belok  kemudi profil   henti  cm  buta  aksi    nama");
    for (uint8_t i = 0; i < RUAS_N; i++) cetakRuas(i);
    Serial.println("  'm7 <idx> <cm>' menyetel panjang ruas.  'm4 <idx>' mulai dari satu ruas.");
    Serial.println("  '?' pada kolom cm = BELUM DIUKUR, misi menolak berangkat.");
}

void Misi::status() {
    Serial.println("\n--- STATUS MISI ---");
    Serial.print("  state       : ");
    switch (_stat) {
        case MISI_DIAM:    Serial.println("DIAM ('m1' mulai, 'm4' lihat tabel)"); break;
        case MISI_PIVOT:   Serial.print("PIVOT -- menghadapkan badan ke ");
                           Serial.print(_nav.namaArah(_arah[_i]));
                           if (ruasSerong(_i)) { Serial.print(" serong ");
                                                 Serial.print(_serong[_i], 0);
                                                 Serial.print(" der"); }
                           Serial.println(); break;
        case MISI_JALAN:   Serial.println("BERJALAN"); break;
        case MISI_SETEL:   Serial.println("MENUNGGU PROFIL GAIT TENANG (badan sedang turun/naik)"); break;
        case MISI_LENGAN:  Serial.println("SEKUENS LENGAN (pose tetap, buta)"); break;
        case MISI_UKUR:    Serial.print("MODE UKUR -- sudah ");
                           Serial.print(_robot.jarakCm() - _ruasAwal, 1);
                           Serial.println(" cm, hentikan di ujung ruas ('s')"); break;
        case MISI_KONFIRM: Serial.println("MENUNGGU KONFIRMASI ('m2' lanjut / 'm3' ulangi)"); break;
        case MISI_SELESAI: Serial.println("SELESAI -- FINISH"); break;
        case MISI_GAGAL:   Serial.println("GAGAL"); break;
        default:           Serial.println("?"); break;
    }
    if (_stat == MISI_GAGAL && _sebab) { Serial.print("  sebab       : "); Serial.println(_sebab); }

    Serial.print("  ruas        : "); Serial.print(_i); Serial.print(" dari 0..");
    Serial.print(RUAS_N - 1); Serial.print("  --  "); Serial.println(RUAS[_i].nama);

    // Sidik tabel ikut di SETIAP status, bukan cuma saat diminta: HUD menarik
    // status berkala, jadi tabel RAM yang tidak lagi sama dengan draft editor
    // (misalnya sesudah Teensy reset) ketahuan tanpa perintah tambahan.
    tabelVersi();

    if (_stat == MISI_JALAN) {
        const Ruas& x = RUAS[_i];
        Serial.print("  di ruas ini : ");
        if (x.henti == HNT_ODO) {
            Serial.print(_robot.jarakCm() - _ruasAwal, 1); Serial.print(" dari ");
            Serial.print(_cm[_i], 0); Serial.println(" cm (odometri gait)");
        } else if (x.henti == HNT_DEPAN) {
            Serial.print("menunggu depan <= "); Serial.print(_cm[_i], 0);
            Serial.print(" cm, sampel "); Serial.print(_n);
            Serial.print("/"); Serial.println(MISI_SAMPEL_N);
        } else if (x.henti == HNT_BELAKANG) {
            Serial.print("menunggu belakang >= "); Serial.print(_blkAwal + _cm[_i], 1);
            Serial.print(" cm, sampel "); Serial.print(_n);
            Serial.print("/"); Serial.println(MISI_SAMPEL_N);
        } else Serial.println("hanya aksi");
        Serial.print("  waktu ruas  : "); Serial.print(lewat() / 1000);
        Serial.print(" dari "); Serial.print(MISI_RUAS_BATAS_MS / 1000); Serial.println(" detik");
    }

    Serial.print("  membawa     : depan ");
    Serial.print(_korban[ARM_DEPAN] ? "KORBAN" : "kosong");
    Serial.print(", belakang ");
    Serial.println(_korban[ARM_BELAKANG] ? "KORBAN" : "kosong");
    Serial.println("  capit       : BELUM TERPASANG -- ruas korban hanya berhenti kosong");

    int depan = _lidar.getDistance(LIDAR_FRONT);
    Serial.print("  depan       : ");
    if      (depan == LIDAR_MATI) Serial.println("MATI -- sensor tidak merespons");
    else if (depan == LIDAR_JAUH) Serial.println("jauh");
    else { Serial.print(depan); Serial.println(" cm"); }

    int blk = _lidar.getDistance(LIDAR_BACK);
    Serial.print("  belakang    : ");
    if      (blk == LIDAR_MATI) Serial.println("MATI");
    else if (blk == LIDAR_JAUH) Serial.println("jauh -- dinding START di luar jangkauan");
    else { Serial.print(blk); Serial.println(" cm dari dinding START"); }

    Serial.print("  navigasi    : ");
    Serial.println(_nav.navMode() == NAV_DIAM ? "DIAM" : "sedang dipegang misi");
    Serial.println("  ('m4' tabel lintasan, 'v' rincian navigasi, 'l' tabel LiDAR)");
}
