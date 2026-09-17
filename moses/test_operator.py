"""python moses/test_operator.py; --preview menyajikan simulasi lokal tanpa perangkat."""
import json
import math
import sys
import time
import types
from operator_control import operator_command, operator_state, parse_operator


class Link:
    hidup = True

    def __init__(self):
        self.log, self.sent = [], []

    def kirim(self, command):
        self.sent.append(command)
        return True


def check():
    link = Link()
    mission = types.SimpleNamespace(manual=False, jeda=False, ganti=lambda *args: None)
    actions = types.SimpleNamespace(batal=lambda: None)
    def send(k, **data):
        operator_command(k, data, link, mission, actions)
    send('op_enter', owner='test')
    assert not mission.manual and not link.sent
    parse_operator(link, '#PROFIL 0 40 60 900 100 70 0')
    send('op_enter', owner='test')
    assert mission.manual and link.sent[-1] == 's'
    send('op_jog', owner='test', seq=1, vector=[1, 1, .2], received=time.monotonic())
    assert link.sent[-1] == 'w0.707 0.707 0.45 0.200'
    send('op_release', owner='test', seq=3)
    assert link.sent[-1] == 's'
    n = len(link.sent)
    send('op_jog', owner='test', seq=2, vector=[1, 0, 0], received=time.monotonic())
    send('op_jog', owner='other', seq=4, vector=[1, 0, 0], received=time.monotonic())
    send('op_jog', owner='test', seq=4, vector=[1, 0, 0], received=time.monotonic()-1)
    send('op_jog', owner='test', seq=5, vector=[math.nan, 0, 0], received=time.monotonic())
    assert len(link.sent) == n
    for values in ([121,60,900,100,70], [40,60,float('inf'),100,70], [40,60], [40,60,900,39,70]):
        send('op_profile', id=0, values=values)
        assert len(link.sent) == n
    send('op_profile', id=2, values=[45,55,1200,90,65])
    assert link.sent[-2:] == ['Tp 2 45.00 55.00 1200.00 90.00 65.00', 'T?']
    send('op_save')
    assert not operator_state(link)['saved'] and operator_state(link)['pending']
    parse_operator(link, '#PROFIL_SIMPAN GAGAL')
    assert not operator_state(link)['saved']
    send('op_save')
    parse_operator(link, '#PROFIL_SIMPAN OK')
    assert operator_state(link)['saved'] and not operator_state(link)['pending']
    parse_operator(link, '#PROFIL_UBAH OK')
    assert not operator_state(link)['saved']
    parse_operator(link, '#PARAM wall.kp 0.012 0 1 0')
    send('op_param', name='wall.kp', value=.02)
    assert link.sent[-1] == 'Qwall.kp 0.02'
    n = len(link.sent)
    send('op_param', name='unknown', value=1)
    send('op_param', name='wall.kp', value=2)
    send('op_param', name='wall.kp', value='nan')
    assert len(link.sent) == n
    link.hidup = False
    send('op_profile', id=1)
    send('op_save')
    assert len(link.sent) == n
    for line in ('#PROFIL', '#PARAM x nan 0 1 0', '#SERVO', '#PROFIL_SIMPAN'):
        parse_operator(link, line)
    print('PASS: kontrol, timeout antrean, urutan STOP, batas profil, parser, dan konfirmasi EEPROM.')


def preview():
    # Adapter uji hanya ada di proses ini; tidak membuka serial/kamera atau EEPROM.
    stub = types.ModuleType('detect')
    for name in ('load_session','letterbox','postprocess','open_camera','CameraThread'):
        setattr(stub, name, lambda *args, **kwargs: None)
    sys.modules['detect'] = stub
    import mission_hud as M
    import threading
    class PreviewLink(M.Teensy):
        @property
        def hidup(self): return True

        @property
        def nama_port(self): return 'SIMULASI LOKAL'

        def kirim(self, command, *args, **kwargs):
            self.log.append('[SIMULASI] '+command)
            if command.startswith('Tp '):
                p=command.split(); self.active=int(p[1]);self.profiles[self.active]=list(map(float,p[2:]));self.mask |= 1<<self.active
                self._parse('#PROFIL_UBAH OK')
            elif len(command)==2 and command[0]=='T' and command[1].isdigit(): self.active=int(command[1])
            elif command=='TW':
                self.saved=json.loads(json.dumps(self.profiles));self._parse('#PROFIL_SIMPAN OK')
            elif command=='TL':
                self.profiles=json.loads(json.dumps(self.saved));self._parse('#PROFIL_MUAT OK')
            elif command.startswith('TD'): self.profiles[int(command[2:])]=[40,60,900,100,70];self._parse('#PROFIL_UBAH OK')
            elif command=='W': self._parse('#CALIB_SIMPAN OK')
            elif command=='q':
                for name, value, lo, hi, effect in [('gait.step_height',40,0,120,1),('gait.cycle_time',900,300,2000,1),('wall.kp',.012,0,1,0),('wall.setpoint',16,5,45,0)]:
                    self._parse(f'#PARAM {name} {value} {lo} {hi} {effect}')
            elif command.startswith('Q'):
                name,value=command[1:].split();p=operator_state(self)['params'][name]
                self._parse(f"#PARAM {name} {value} {p['min']} {p['max']} {p['effect']}")
            elif command=='Yt':
                for i in range(18):self._parse(f'#TRIM {i} K{i//3}_{["COXA","FEMUR","TIBIA"][i%3]} 0 0')
            if command.startswith('T'):
                self._parse(f'#PROFIL {self.active} '+ ' '.join(map(str,self.profiles[self.active]))+f' {self.mask}')
                self._parse('#SERVO 1')
            return True
    link=PreviewLink(None);link.active=0;link.mask=0
    link.profiles=[[40,60,900,100,70],[75,70,1100,115,70],[40,45,1100,80,70],[30,45,1000,100,45],[40,60,1300,100,70],[75,45,1300,100,60]]
    link.saved=json.loads(json.dumps(link.profiles))
    kalib=M.Kalib();misi=M.Misi();aksi=M.Aksi(link);juri=M.Juri(kalib,{})
    bersama=M.Bersama()
    def loop():
        while True:
            for k,v in bersama.ambil_perintah():
                if k=='stop':misi.halt=True;misi.manual=False;link.kirim('s')
                elif k=='lepas_stop':misi.halt=False
                elif k.startswith('op_') and (not misi.halt or k in ('op_read','op_release','op_exit')):operator_command(k,v,link,misi,aksi)
                elif k=='man':link.kirim(v)
            d=M.rakit_state(misi,link,kalib,juri,{'fps':0,'t_inf':0},True,'Pratinjau lokal',bersama=bersama)
            d['galat']='SIMULASI LOKAL · tanpa robot, kamera, atau penulisan EEPROM sungguhan.'
            bersama.set_state(d)
            time.sleep(.05)
    threading.Thread(target=loop,daemon=True).start()
    print('Pratinjau simulasi: http://127.0.0.1:8765',flush=True)
    M.ThreadingHTTPServer(('127.0.0.1',8765),M.buat_handler(bersama)).serve_forever()


if __name__=='__main__':
    preview() if '--preview' in sys.argv else check()
