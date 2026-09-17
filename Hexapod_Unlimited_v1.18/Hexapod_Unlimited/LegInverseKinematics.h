#ifndef LEGINVERSEKINEMATICS_H
#define LEGINVERSEKINEMATICS_H

#include "config.h"
#include <math.h>
#include "types.h"

class LegInverseKinematics {
public:
  // kunciLututDeg = NAN (baku) -> IK tiga sendi biasa.
  //
  // Angka apa pun selain NAN MENGUNCI lutut di sudut itu. Dengan lutut tetap,
  // jarak sendi-femur -> telapak juga tetap, jadi telapak tidak lagi bisa
  // berada di sembarang titik: ia jatuh ke BUSUR berjari-jari tetap, pada
  // ARAH yang diminta. Komponen radial perintah hilang; komponen sudutnya
  // utuh. Dipakai profil KAIL untuk kaki depan -- lutut yang mengayuh
  // menggeser titik sentuh ujung kaki dan membuatnya menggaruk di anak
  // tangga. Lihat KAIL_LUTUT_KUNCI di config.h.
  static bool solve(float x, float y, float z, float& coxaDeg, float& femurDeg, float& tibiaDeg,
                    float kunciLututDeg = NAN);
};

#endif
