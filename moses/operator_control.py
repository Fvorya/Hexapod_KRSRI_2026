"""Kontrol operator dan balasan firmware; tanpa kamera atau dependensi UI."""
import math
import time
import json

PROFILE_LIMITS = ((0, 120), (0, 150), (300, 3000), (40, 160), (30, 120))


def operator_state(link):
    if not hasattr(link, "operator"):
        link.operator = {"profile": None, "params": {}, "servo": None,
                         "message": "Baca robot untuk mengambil profil dan kalibrasi.",
                         "reply": 0, "pending": 0, "saved": False}
    return link.operator


def parse_operator(link, line):
    state = operator_state(link)
    words = line.split()
    if not words:
        return
    if "DITOLAK" in line.upper() or "DI LUAR JANGKAUAN IK" in line.upper():
        state["message"] = "Robot: " + line.strip()
    if (line.startswith("Trim TERSIMPAN") or line.startswith("GAGAL menyimpan ServoMap")
            or line.startswith("Kompas Arena disimpan") or line.startswith("Navigation: kalibrasi pivot disimpan")):
        state["message"] = line.strip()
    try:
        if words[0] == "#PROFIL" and len(words) == 8:
            values = [float(v) for v in words[2:7]]
            if not all(math.isfinite(v) for v in values):
                return
            previous = state["profile"]
            if previous and previous["id"] == int(words[1]) and previous["values"] != values:
                state["saved"] = False
            state["profile"] = {"id": int(words[1]), "values": values,
                                "mask": int(words[7])}
            state["read_at"] = time.monotonic()
        elif words[0] == "#SERVO" and words[1] in ("0", "1"):
            state["servo"] = words[1] == "1"
        elif words[0] == "#PARAM" and len(words) == 6:
            numbers = [float(v) for v in words[2:5]]
            if all(math.isfinite(v) for v in numbers):
                state["params"][words[1]] = dict(zip(("value", "min", "max"), numbers),
                                                effect=int(words[5]))
        elif words[0] in ("#PROFIL_SIMPAN", "#PROFIL_MUAT", "#PROFIL_UBAH", "#CALIB_SIMPAN"):
            action = {"#PROFIL_SIMPAN": "Profil tersimpan di EEPROM dan terverifikasi",
                      "#PROFIL_MUAT": "Profil dimuat dari EEPROM",
                      "#PROFIL_UBAH": "Profil diterapkan di RAM; belum disimpan ke EEPROM",
                      "#CALIB_SIMPAN": "Kalibrasi tersimpan di EEPROM dan terverifikasi"}[words[0]]
            state["message"] = action if words[1] == "OK" else "Gagal: " + words[0][1:].lower().replace("_", " ")
            state["reply"] += 1
            state["pending"] = 0
            state["saved"] = words[1] == "OK" and words[0] in ("#PROFIL_SIMPAN", "#PROFIL_MUAT")
            if words[0] == "#PROFIL_MUAT" and words[1] == "OK":
                state["profile"] = None
    except (ValueError, IndexError):
        return


def operator_snapshot(link):
    state = dict(operator_state(link))
    state["pending_age"] = time.monotonic() - state["pending"] if state["pending"] else 0
    return state


def operator_command(k, payload, link, misi, aksi):
    """Validasi sebelum mengirim; jog kedaluwarsa tidak boleh dimainkan ulang."""
    state = operator_state(link)
    try:
        data = json.loads(payload) if isinstance(payload, str) else (payload or {})
        if not isinstance(data, dict):
            raise ValueError("Format kontrol tidak sah.")
        if not link.hidup:
            raise ValueError("Teensy belum tersambung. Tidak ada perintah yang dikirim.")
        if k == "op_read":
            for cmd in ("T?", "q", "k", "Yt"):
                link.kirim(cmd)
            state["message"] = "Meminta data robot. Jika profil tetap kosong, perbarui firmware Teensy."
            return
        if k == "op_enter":
            if not state["profile"]:
                raise ValueError("Baca robot dahulu; kontrol ini memerlukan firmware dengan dukungan profil.")
            owner = str(data.get("owner", ""))
            if not owner or len(owner) > 80:
                raise ValueError("Sesi kontrol tidak sah.")
            if getattr(misi, "manual", False) and misi.operator_owner != owner:
                raise ValueError("Mode manual sedang dipakai tab lain. Gunakan STOP untuk mengambil alih.")
            aksi.batal()
            link.kirim("s")
            misi.ganti("IDLE", "mode manual")
            misi.manual = True
            misi.jeda = False
            misi.operator_owner, misi.operator_seq = owner, -1
            state["message"] = "Manual aktif. Tekan-tahan arah untuk bergerak."
            return
        if k in ("op_jog", "op_release", "op_exit"):
            if not getattr(misi, "manual", False) or data.get("owner") != misi.operator_owner:
                raise ValueError("Aktifkan mode manual dari tab ini dahulu.")
            seq = data.get("seq")
            if type(seq) is not int or seq <= misi.operator_seq:
                return
            misi.operator_seq = seq
            if k != "op_jog":
                link.kirim("s")
                if k == "op_exit":
                    misi.manual = False
                return
            if time.monotonic() - data.get("received", 0) > 0.35:
                return
            vec = data.get("vector")
            if not isinstance(vec, list) or len(vec) != 3:
                raise ValueError("Vektor gerak harus berisi tiga angka.")
            f, s, t = [float(v) for v in vec]
            if not all(math.isfinite(v) and -1 <= v <= 1 for v in (f, s, t)):
                raise ValueError("Vektor gerak di luar batas.")
            norm = max(1, math.hypot(f, s))
            link.kirim(f"w{f/norm:.3f} {s/norm:.3f} 0.45 {t:.3f}")
            return
        if not getattr(misi, "manual", False):
            raise ValueError("Masuk mode manual sebelum menyetel robot.")
        if k == "op_profile":
            idx = data.get("id")
            if type(idx) is not int or not 0 <= idx <= 5:
                raise ValueError("Pilih salah satu dari enam profil.")
            values = data.get("values")
            if values is None:
                link.kirim(f"T{idx}")
            else:
                if not isinstance(values, list) or len(values) != 5:
                    raise ValueError("Profil harus berisi lima nilai.")
                values = [float(v) for v in values]
                if not all(math.isfinite(v) and lo <= v <= hi
                           for v, (lo, hi) in zip(values, PROFILE_LIMITS)):
                    raise ValueError("Nilai profil di luar rentang yang diizinkan.")
                link.kirim(f"Tp {idx} " + " ".join(f"{v:.2f}" for v in values))
            state["saved"] = False
            state["message"] = "Meminta profil diterapkan; menunggu nilai dari robot."
            link.kirim("T?")
        elif k in ("op_save", "op_load", "op_defaults", "op_calib_save"):
            if state["pending"] and time.monotonic() - state["pending"] < 5:
                raise ValueError("Masih menunggu balasan EEPROM sebelumnya.")
            command = {"op_save": "TW", "op_load": "TL", "op_calib_save": "W"}.get(k)
            if k == "op_defaults":
                idx = data.get("id")
                if type(idx) is not int or not 0 <= idx <= 5:
                    raise ValueError("Profil tidak sah.")
                command = f"TD{idx}"
            state["pending"] = time.monotonic()
            state["message"] = "Menunggu konfirmasi firmware; belum dinyatakan berhasil."
            link.kirim(command)
        elif k == "op_param":
            name, value = data.get("name"), float(data.get("value"))
            p = state["params"].get(name)
            if not p or not math.isfinite(value) or not p["min"] <= value <= p["max"]:
                raise ValueError("Baca kalibrasi dahulu dan isi nilai di dalam rentang.")
            link.kirim(f"Q{name} {value:.6g}")
            state["message"] = "Parameter dikirim; periksa nilai balasan sebelum menyimpan."
        else:
            raise ValueError("Perintah operator tidak dikenal.")
    except (ValueError, TypeError, KeyError, OverflowError) as exc:
        state["message"] = str(exc)
        link.log.append("[OPERATOR] " + str(exc))
