#!/usr/bin/env python3
"""
controller_orin.py — HUNTER gerçek donanım Python beyni
=======================================================
controller.py (SITL) sürümünün gerçek-HW + mavlink-router uyarlaması.

MIMARI (mavlink-router):
  runtracker (C++)  --TCP FRAME 9999-->  bu kod (bbox)
  CUAV V5+  --USB-->  mavlink-router  --UDP 14551-->  bu kod (ATTITUDE+RC oku, RC_OVERRIDE yaz)

controller.py'den FARKLAR:
  - uinput / VirtualJoystick KALDIRILDI  (manuel uçuş MK15->alıcı->FC doğrudan, SITL artifact'iydi)
  - MK15Reader KALDIRILDI  (RC artık router'dan RC_CHANNELS olarak gelir)
  - SITLConnection -> FCConnection  (endpoint udpout:127.0.0.1:14551, RC_CHANNELS okur, autopilot sys'e kilitli)
  - StateMachine SADELEŞTİ  (runtracker'a SHOW_ROI/TRACK_START komutu GÖNDERMEZ — RC'den runtracker yapar;
    sadece FRAME state'inden tracker durumunu okur + CH6 ile otonom kapısını yönetir)
  - mpc_teacher PRIVILEGED'SİZ çağrılır  (GPS-free; default mpc_teacher1 — W_real YOK, sadece bbox-delta)
  - GÜVENLİK: varsayılan DRY-RUN (sadece oku+logla). RC_OVERRIDE için --enable-override.

KORUNAN (controller.py ile birebir):
  obs montajı, action_to_pwm, AUTO_*_DIR, send_rc_override/release, setup_acro_params, FlightLogger.

KULLANIM:
  # Faz 3 — log only (motor YOK, sadece obs/action doğrula):
  python3 controller_orin.py
  # Faz 4 — RC_OVERRIDE canlı (PERVANESİZ bench):
  python3 controller_orin.py --enable-override
  # Farklı kontrolcü (dosya adı değiştir):
  python3 controller_orin.py --teacher mpc_teacher2
"""

import argparse
import importlib
import socket
import sys
import time
import math
import threading
import numpy as np

try:
    from pymavlink import mavutil
except ImportError:
    print("HATA: pymavlink yok. pip3 install --user pymavlink")
    sys.exit(1)


# ============================================================================
# SABİTLER
# ============================================================================
CAMERA_HOST = '127.0.0.1'        # runtracker FRAME server (Python Orin'de → localhost)
CAMERA_PORT = 9999

# mavlink-router brain endpoint. udpout: ilk paketimiz router'a adresimizi öğretir.
FC_ENDPOINT = 'udpout:127.0.0.1:14551'

# Switch eşikleri (3-pos)
SWITCH_LOW_MAX  = 1300
SWITCH_HIGH_MIN = 1700

# Kamera FOV — FRAME'deki ang_x/ang_y zaten runtracker'da bu FOV ile hesaplanıyor.
# Burada sadece referans; obs ang değerleri doğrudan FRAME'den gelir.
CAMERA_HFOV = 80.0
CAMERA_VFOV = 50.534

# Kontrol döngüsü — mpc_teacher 30 Hz'e göre tunelenmiş (rate_limit'ler 30Hz). Değiştirme.
LOOP_HZ = 30

# ===========================================================================
# AUTONOMOUS RC EKSEN YÖNÜ
# ===========================================================================
# RC_OVERRIDE, FC'nin RC kalibrasyonunu ATLAR → ham eksen yönü geçerli.
# DİKKAT: bu değerler SİM'den (SITL) geliyor. GERÇEK DONANIMDA YENİDEN ÖLÇÜLMELİ
#         (Faz 4: pervanesiz, RC_OVERRIDE ver, motor yönü doğru mu bak).
#  +1 = düz,  -1 = ters
AUTO_ROLL_DIR     = +1
AUTO_PITCH_DIR    = -1   # SİM'de ters çıkmıştı — gerçek HW'de TEYİT ET
AUTO_THROTTLE_DIR = +1
AUTO_YAW_DIR      = +1


# ============================================================================
# YARDIMCI FONKSİYONLAR
# ============================================================================

def switch_state(rc_value):
    """3-pos switch: 0=LOW, 1=MID, 2=HIGH"""
    if rc_value == 0 or rc_value == 65535:
        return 1  # bilinmiyor → MID kabul
    if rc_value < SWITCH_LOW_MAX:
        return 0
    elif rc_value > SWITCH_HIGH_MIN:
        return 2
    else:
        return 1


def action_to_pwm(action_val):
    """mpc_teacher action (-1..+1) → RC PWM (1000-2000). camera_bridge ile aynı formül."""
    val = max(-1.0, min(1.0, float(action_val)))
    pwm = int(val * 500.0 + 1500.0)
    return max(1000, min(2000, pwm))


# ============================================================================
# FLIGHT LOGGER — her frame obs+action CSV (replay_compare.py için)
# ============================================================================

class FlightLogger:
    HEADER = [
        't', 'mode',
        'obs_cx', 'obs_cy', 'obs_bbox_w', 'obs_bbox_h',
        'obs_ang_x', 'obs_ang_y',
        'obs_roll_deg', 'obs_pitch_deg', 'obs_yaw_deg',
        'obs_roll_rate', 'obs_pitch_rate', 'obs_yaw_rate',
        'obs_alt', 'obs_bbox_valid',
        'act_roll', 'act_pitch', 'act_throttle', 'act_yaw',
        'pwm_roll', 'pwm_pitch', 'pwm_throttle', 'pwm_yaw',
    ]

    def __init__(self, path):
        self.path = path
        self.file = None
        self.t0 = time.time()
        self.row_count = 0

    def open(self):
        try:
            self.file = open(self.path, 'w')
            self.file.write(','.join(self.HEADER) + '\n')
            print(f"[LOG] Flight log açıldı: {self.path}")
            return True
        except Exception as e:
            print(f"[LOG] HATA: log açılamadı: {e}")
            return False

    def log(self, mode, obs, action, pwm):
        if self.file is None:
            return
        t = time.time() - self.t0
        row = [f"{t:.4f}", mode]
        row += [f"{float(x):.6f}" for x in obs]
        row += [f"{float(x):.6f}" for x in action]
        row += [str(int(x)) for x in pwm]
        self.file.write(','.join(row) + '\n')
        self.row_count += 1
        # Her frame flush ETME (disk I/O döngüyü takıyor → telemetri birikip
        # CH6 tepkisini geciktiriyor). 30 satırda bir flush yeter.
        if self.row_count % 30 == 0:
            self.file.flush()

    def close(self):
        if self.file:
            self.file.close()
            print(f"[LOG] Flight log kapatıldı: {self.row_count} satır → {self.path}")


# ============================================================================
# RUNTRACKER FRAME TCP CLIENT  (camera_bridge protokolü ile birebir)
# ============================================================================

class FrameClient:
    """runtracker.cpp TCP FRAME server'ından bbox okur. Komut GÖNDERMEZ (RC'den runtracker yapar)."""

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.connected = False
        self.recv_buffer = ""
        self.last_bbox = {
            'valid': False, 'cx': 0.5, 'cy': 0.5, 'w': 0.0, 'h': 0.0,
            'ang_x': 0.0, 'ang_y': 0.0, 'state': 'idle', 'frame_id': 0,
        }
        self._lock = threading.Lock()

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.host, self.port))
            self.sock.setblocking(False)
            self.connected = True
            print(f"[FRAME] runtracker'a bağlandı: {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"[FRAME] HATA: bağlantı kurulamadı: {e}")
            return False

    def poll(self):
        if not self.connected:
            return
        try:
            data = self.sock.recv(4096)
            if not data:
                print("[FRAME] runtracker bağlantıyı kapattı")
                self.connected = False
                return
            self.recv_buffer += data.decode('utf-8', errors='ignore')
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"[FRAME] Recv hatası: {e}")
            self.connected = False
            return

        while '\n' in self.recv_buffer:
            line, self.recv_buffer = self.recv_buffer.split('\n', 1)
            line = line.strip()
            if line:
                self._parse_frame(line)

    def _parse_frame(self, line):
        """FRAME <id> <valid> <cx> <cy> <w> <h> <ang_x> <ang_y> <state>"""
        parts = line.split()
        if len(parts) < 10 or parts[0] != 'FRAME':
            return
        try:
            with self._lock:
                self.last_bbox = {
                    'frame_id': int(parts[1]),
                    'valid': int(parts[2]) == 1,
                    'cx': float(parts[3]),
                    'cy': float(parts[4]),
                    'w': float(parts[5]),
                    'h': float(parts[6]),
                    'ang_x': float(parts[7]),
                    'ang_y': float(parts[8]),
                    'state': parts[9],
                }
        except (ValueError, IndexError):
            pass

    def get_bbox(self):
        with self._lock:
            return dict(self.last_bbox)

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass


# ============================================================================
# STATE MACHINE  (gerçek-HW: sadece CH6 otonom kapısı + FRAME tracker sync)
# ============================================================================

class StateMachine:
    """
    Gerçek donanımda runtracker ROI/track'i RC'den (CH5/CH12/CH13) KENDİ yapar.
    Bu yüzden burada SADECE:
      - tracker_active'i FRAME state'inden oku (komut göndermeden)
      - CH6 ile manual <-> autonomous gating (MID güvenlikli)
    """

    def __init__(self, frame_client):
        self.cam = frame_client
        self.ch6_has_been_mid = False
        self.tracker_active = False
        self.mode = 'manual'   # 'manual' | 'autonomous'

    def process_rc(self, channels):
        ch6 = channels.get('ch6', 1500)

        # --- Tracker durumu FRAME'den (runtracker'ın gerçek hali) ---
        bbox = self.cam.get_bbox()
        cam_state = bbox.get('state', 'idle')
        self.tracker_active = (cam_state == 'tracking')

        # --- CH6: manual <-> autonomous (MID güvenlikli) ---
        ch6_state = switch_state(ch6)

        # MID veya LOW görüldü → güvenlik kalkar + otonomdan manuele dön
        if ch6_state == 1 or ch6_state == 0:
            self.ch6_has_been_mid = True
            if self.mode == 'autonomous':
                self.mode = 'manual'
                print("[STATE] CH6 HIGH'tan ayrıldı → MANUAL")

        # HIGH → autonomous (güvenlik kalktıysa + tracker aktifse)
        if ch6_state == 2 and self.ch6_has_been_mid:
            if self.mode == 'manual':
                if self.tracker_active:
                    self.mode = 'autonomous'
                    print("[STATE] CH6 HIGH → AUTONOMOUS")
                else:
                    print("[STATE] CH6 HIGH ama tracker aktif değil, autonomous'a geçmiyor")

        # Tracker düşerse (lost/idle) otonomdan çık
        if self.mode == 'autonomous' and not self.tracker_active:
            self.mode = 'manual'
            print(f"[STATE] tracker durdu ({cam_state}) → MANUAL")

        return ch6_state


# ============================================================================
# FC CONNECTION  (mavlink-router üzerinden — telemetri+RC oku, RC_OVERRIDE yaz)
# ============================================================================

class FCConnection:
    """mavlink-router'ın UDP endpoint'i ile MAVLink. RC_CHANNELS + ATTITUDE okur."""

    def __init__(self, endpoint):
        self.endpoint = endpoint
        self.conn = None
        self.target_sys = 1
        self.target_comp = 1

        # Telemetri (obs için)
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.roll_rate = 0.0     # rad/s (ATTITUDE)
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0
        self.alt_rel = 0.0
        self.last_attitude_time = 0.0

        # RC (state machine için) — router'dan RC_CHANNELS
        self.rc_channels = {f'ch{i}': 1500 for i in range(1, 17)}
        self.last_rc_time = 0.0

    def connect(self):
        print(f"[FC] Bağlanıyor: {self.endpoint}")
        try:
            self.conn = mavutil.mavlink_connection(self.endpoint, source_system=255, source_component=190)
        except Exception as e:
            print(f"[FC] HATA: bağlantı açılamadı: {e}")
            return False

        # Router'a ilk paket → adresimizi öğrensin; ArduPilot heartbeat'ini bekle
        # (router'da runtracker da heartbeat atıyor → SADECE gerçek autopilot'a kilitlen)
        print("[FC] ArduPilot heartbeat bekleniyor...")
        self.conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

        t0 = time.time()
        ap_hb = None
        while time.time() - t0 < 15:
            hb = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if hb is None:
                continue
            # GCS değil, gerçek autopilot mı?
            if hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                ap_hb = hb
                break
        if ap_hb is None:
            print("[FC] HATA: ArduPilot heartbeat alınamadı! (router çalışıyor mu?)")
            return False

        self.target_sys = ap_hb.get_srcSystem()
        self.target_comp = ap_hb.get_srcComponent()
        print(f"[FC] Bağlandı — autopilot sys={self.target_sys} comp={self.target_comp}")

        # Stream iste — ArduPilot USB'de otomatik göndermiyor
        self._request_streams()
        return True

    def _request_streams(self):
        try:
            self.conn.mav.request_data_stream_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 30, 1)        # ATTITUDE
            self.conn.mav.request_data_stream_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 20, 1)   # RC_CHANNELS
            self.conn.mav.request_data_stream_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)      # GLOBAL_POSITION_INT
        except Exception as e:
            print(f"[FC] Uyarı: stream isteği başarısız: {e}")

    def setup_acro_params(self):
        """
        ACRO modu kritik parametreleri. Bunlar olmadan drone stick merkeze
        gelince auto-level yapar → mpc_teacher'ın küçük action'ları işe yaramaz.
        SADECE --enable-override ile çağrılır (FC paramı değiştirir).
        """
        acro_params = {
            'ACRO_OPTIONS':   2,
            'ACRO_RP_EXPO':   0,
            'ACRO_Y_EXPO':    0,
            'ACRO_TRAINER':   0,
            'ACRO_BAL_ROLL':  0,
            'ACRO_BAL_PITCH': 0,
            'RC1_DZ':         0,
            'RC2_DZ':         0,
            'RC3_DZ':         0,
            'RC4_DZ':         0,
        }
        print("[FC] ACRO parametreleri ayarlanıyor...")
        for name, value in acro_params.items():
            try:
                self.conn.mav.param_set_send(
                    self.target_sys, self.target_comp,
                    name.encode('ascii'), float(value),
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                time.sleep(0.05)
            except Exception as e:
                print(f"[FC] Uyarı: {name} ayarlanamadı: {e}")
        print(f"[FC] {len(acro_params)} ACRO parametresi gönderildi")

    def send_heartbeat(self):
        """GCS heartbeat — router route'unu canlı tut (dry-run'da da güvenli, sadece duyuru)."""
        if self.conn is None:
            return
        try:
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except Exception:
            pass

    def poll(self):
        """ATTITUDE + GLOBAL_POSITION_INT + RC_CHANNELS oku."""
        if self.conn is None:
            return
        while True:
            msg = self.conn.recv_match(
                type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'RC_CHANNELS'],
                blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if t == 'ATTITUDE':
                self.roll_deg = math.degrees(msg.roll)
                self.pitch_deg = math.degrees(msg.pitch)
                self.yaw_deg = math.degrees(msg.yaw)
                self.roll_rate = msg.rollspeed   # rad/s
                self.pitch_rate = msg.pitchspeed
                self.yaw_rate = msg.yawspeed
                self.last_attitude_time = time.time()
            elif t == 'GLOBAL_POSITION_INT':
                self.alt_rel = msg.relative_alt / 1000.0
            elif t == 'RC_CHANNELS':
                self.rc_channels = {
                    'ch1': msg.chan1_raw, 'ch2': msg.chan2_raw, 'ch3': msg.chan3_raw,
                    'ch4': msg.chan4_raw, 'ch5': msg.chan5_raw, 'ch6': msg.chan6_raw,
                    'ch7': msg.chan7_raw, 'ch8': msg.chan8_raw, 'ch9': msg.chan9_raw,
                    'ch10': msg.chan10_raw, 'ch11': msg.chan11_raw, 'ch12': msg.chan12_raw,
                    'ch13': msg.chan13_raw, 'ch14': msg.chan14_raw, 'ch15': msg.chan15_raw,
                    'ch16': msg.chan16_raw,
                }
                self.last_rc_time = time.time()

    def send_rc_override(self, roll, pitch, throttle, yaw):
        """action (-1..+1) → RC_OVERRIDE PWM. AUTO_*_DIR uygulanır. CH1-4 override, gerisi RC."""
        if self.conn is None:
            return
        rc_roll     = action_to_pwm(AUTO_ROLL_DIR     * roll)
        rc_pitch    = action_to_pwm(AUTO_PITCH_DIR    * pitch)
        rc_throttle = action_to_pwm(AUTO_THROTTLE_DIR * throttle)
        rc_yaw      = action_to_pwm(AUTO_YAW_DIR      * yaw)
        self.conn.mav.rc_channels_override_send(
            self.target_sys, self.target_comp,
            rc_roll, rc_pitch, rc_throttle, rc_yaw,
            0, 0, 0, 0,                      # CH5-8: 0 = override etme (RC kullan)
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0)    # CH9-18

    def release_rc_override(self):
        """
        Tüm override iptal — ANINDA RC'ye dön.
        KRİTİK: MAVLink'te 0 = 'kanalı serbest bırak (RC'ye dön)',
                65535 = 'yok say (override'ı TUT)'. Release için 0 GÖNDER.
        Eskiden 65535 gönderiyordu → release etmiyordu, sadece RC_OVERRIDE_TIME
        timeout'u (saniyeler) ile dönüyordu. 0 ile anlık döner.
        """
        if self.conn is None:
            return
        self.conn.mav.rc_channels_override_send(
            self.target_sys, self.target_comp,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    def release_burst(self, n=5):
        """Release'i n kez arka arkaya gönder — UDP paket kaybına karşı güvence (anlık kesinti)."""
        for _ in range(max(1, n)):
            self.release_rc_override()


# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="HUNTER gerçek-HW Python beyni")
    parser.add_argument('--cam-host', default=CAMERA_HOST,
                        help="runtracker FRAME host (default 127.0.0.1)")
    parser.add_argument('--cam-port', type=int, default=CAMERA_PORT)
    parser.add_argument('--fc', default=FC_ENDPOINT,
                        help="mavlink-router endpoint (default udpout:127.0.0.1:14551)")
    parser.add_argument('--teacher', default='mpc_teacher1',
                        help="Kontrolcü modülü (dosya adı, .py'siz). Örn: mpc_teacher1, mpc_teacher2")
    parser.add_argument('--enable-override', action='store_true',
                        help="RC_OVERRIDE'ı GERÇEKTEN gönder + ACRO param set et "
                             "(PERVANESİZ bench!). Verilmezse DRY-RUN: sadece oku+logla.")
    parser.add_argument('--no-fc', action='store_true',
                        help="FC'ye bağlanma (sadece FRAME testi için)")
    parser.add_argument('--log', default=None,
                        help="Flight log CSV (verilmezse otomatik tarih-saat)")
    args = parser.parse_args()

    dry_run = not args.enable_override

    print("=" * 70)
    print(" controller_orin.py — Gerçek Donanım Beyni")
    print("=" * 70)
    if dry_run:
        print(" MOD: DRY-RUN (sadece oku+logla, RC_OVERRIDE YOK) — Faz 3")
    else:
        print(" MOD: !!! RC_OVERRIDE CANLI !!! — PERVANELER ÇIKIK OLMALI — Faz 4")
    print("=" * 70)

    # --- Kontrolcü yükle (modüler swap) ---
    try:
        tmod = importlib.import_module(args.teacher)
        TeacherClass = getattr(tmod, 'HybridTeacher', None) or getattr(tmod, 'PNTeacher')
    except Exception as e:
        print(f"[!] Kontrolcü '{args.teacher}' yüklenemedi: {e}")
        sys.exit(1)
    teacher = TeacherClass()
    teacher.reset()
    print(f"[CTRL] Kontrolcü hazır: {args.teacher}.{TeacherClass.__name__} "
          f"(privileged'siz → GPS-free, hedef-boyutsuz)")

    # --- runtracker FRAME bağlan ---
    cam = FrameClient(args.cam_host, args.cam_port)
    if not cam.connect():
        print("[!] runtracker FRAME'e bağlanılamadı. runtracker (kcfTracker) çalışıyor mu?")
        sys.exit(1)

    # --- FC (router) bağlan ---
    fc = None
    if not args.no_fc:
        fc = FCConnection(args.fc)
        if not fc.connect():
            print("[!] FC'ye (router) bağlanılamadı. mavlink-routerd çalışıyor mu? (14551)")
            sys.exit(1)
        if not dry_run:
            fc.setup_acro_params()   # SADECE override modunda FC paramı değiştir
        else:
            print("[FC] DRY-RUN: ACRO param SET edilmedi (FC'ye yazım yok).")

    state = StateMachine(cam)

    log_path = args.log or time.strftime("flight_%Y%m%d_%H%M%S.csv")
    logger = FlightLogger(log_path)
    logger.open()

    print("\n" + "=" * 70)
    print("Çalışıyor... CTRL+C ile çık.")
    print("=" * 70 + "\n")

    last_print = time.time()
    last_loop = time.time()
    last_hb = time.time()
    loop_period = 1.0 / LOOP_HZ
    last_action = [0.0, 0.0, 0.0, 0.0]
    was_overriding = False   # güvenlik: override→manuel geçiş kenarını yakala

    try:
        while True:
            now = time.time()

            cam.poll()
            if fc is not None:
                fc.poll()
                if now - last_hb >= 1.0:
                    fc.send_heartbeat()      # route'u canlı tut (dry-run'da da güvenli)
                    last_hb = now

            # RC kaynağı: router RC_CHANNELS (mk15 yok)
            channels = fc.rc_channels if fc else {f'ch{i}': 1500 for i in range(1, 17)}
            state.process_rc(channels)

            # ============ GÜVENLİK: override kapısı ============
            # state.mode'a TEK BAŞINA güvenme. Override için ÜÇ şart birden:
            #   (1) state otonom, (2) CH6 HÂLÂ HIGH (doğrudan PWM — anlık),
            #   (3) hedef geçerli. Üçünden biri düşerse override ANINDA kesilir.
            bbox = cam.get_bbox()
            ch6_high = channels.get('ch6', 1500) > SWITCH_HIGH_MIN
            override_ok = (state.mode == 'autonomous') and ch6_high and bbox['valid']

            if override_ok:
                obs = np.zeros(14, dtype=np.float32)
                obs[0] = bbox['cx'];  obs[1] = bbox['cy']
                obs[2] = bbox['w'];   obs[3] = bbox['h']
                obs[4] = bbox['ang_x']; obs[5] = bbox['ang_y']
                obs[6] = fc.roll_deg if fc else 0.0
                obs[7] = fc.pitch_deg if fc else 0.0
                obs[8] = fc.yaw_deg if fc else 0.0
                # MAVLink ATTITUDE rate'leri RAD/s → mpc_teacher DERECE/s bekliyor
                obs[9]  = math.degrees(fc.roll_rate) if fc else 0.0
                obs[10] = math.degrees(fc.pitch_rate) if fc else 0.0
                obs[11] = math.degrees(fc.yaw_rate) if fc else 0.0
                obs[12] = fc.alt_rel if fc else 0.0
                obs[13] = 1.0

                action = teacher.compute_action(obs)   # privileged YOK → GPS-free
                last_action = action
                pwm = (action_to_pwm(action[0]), action_to_pwm(action[1]),
                       action_to_pwm(action[2]), action_to_pwm(action[3]))

                if not dry_run and fc is not None:
                    fc.send_rc_override(action[0], action[1], action[2], action[3])
                logger.log('autonomous' if not dry_run else 'auto_dry', obs, action, pwm)
                was_overriding = True
            else:
                # Override AKTİF DEĞİL → manuele dön. state hâlâ 'autonomous' diyorsa zorla düşür.
                if state.mode == 'autonomous':
                    state.mode = 'manual'
                # Override'dan YENİ çıktıysak: ANINDA + TEKRARLI release (UDP güvencesi) + zaman damgası
                if was_overriding:
                    if not dry_run and fc is not None:
                        fc.release_burst(5)        # 5 paket — biri kaybolsa da geçer
                    reason = ("CH6 LOW" if not ch6_high else
                              ("hedef kayip" if not bbox['valid'] else "state"))
                    print(f"[SAFE] {time.strftime('%H:%M:%S')} OVERRIDE KESILDI ({reason}) -> MANUAL")
                    was_overriding = False
                else:
                    # Manuelde her dongu release gonder — override ASLA geri sizmasin
                    if not dry_run and fc is not None:
                        fc.release_rc_override()

            # --- Periyodik durum ---
            if now - last_print >= 1.0:
                bbox = cam.get_bbox()
                ch5 = channels.get('ch5', 0); ch6 = channels.get('ch6', 0)
                ch12 = channels.get('ch12', 0); ch13 = channels.get('ch13', 0)
                cam_state = bbox.get('state', 'idle')
                if fc is None:
                    tel = "no-fc"
                elif fc.last_attitude_time == 0.0:
                    tel = "ATT-YOK!"
                else:
                    tel = f"{now - fc.last_attitude_time:.1f}s"
                rc_age = "RC-YOK!" if (fc and fc.last_rc_time == 0.0) else "ok"
                tag = "DRY" if dry_run else "LIVE"
                print(f"[{tag}|{state.mode:10s}] CH5={ch5} CH6={ch6} CH12={ch12} CH13={ch13} "
                      f"| TRK={cam_state:8s} BBOX={'Y' if bbox.get('valid') else 'N'} "
                      f"| att={tel} rc={rc_age} "
                      f"r={fc.roll_deg if fc else 0:+.1f} p={fc.pitch_deg if fc else 0:+.1f}")
                if state.mode == 'autonomous':
                    a = last_action
                    pr = action_to_pwm(AUTO_ROLL_DIR * a[0]); pp = action_to_pwm(AUTO_PITCH_DIR * a[1])
                    pt = action_to_pwm(AUTO_THROTTLE_DIR * a[2]); py = action_to_pwm(AUTO_YAW_DIR * a[3])
                    arrow = "→ PWM" if not dry_run else "→ (DRY, gönderilmedi) PWM"
                    print(f"             └─ ACTION r={a[0]:+.3f} p={a[1]:+.3f} t={a[2]:+.3f} y={a[3]:+.3f} "
                          f"{arrow} [{pr},{pp},{pt},{py}] | ang_x={bbox.get('ang_x',0):+.1f} "
                          f"ang_y={bbox.get('ang_y',0):+.1f} bbox_w={bbox.get('w',0):.4f}")
                last_print = now

            elapsed = time.time() - last_loop
            time.sleep(max(0.0, loop_period - elapsed))
            last_loop = time.time()

    except KeyboardInterrupt:
        print("\n\nÇıkış sinyali...")

    # Cleanup
    if not dry_run and fc is not None:
        fc.release_rc_override()
    logger.close()
    cam.close()
    print("Kapatıldı.")


if __name__ == "__main__":
    main()
