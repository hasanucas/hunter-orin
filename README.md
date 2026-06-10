# HUNTER — Orin Gerçek Donanım Stack

Otonom FPV drone hedef takip + müdahale sisteminin **gerçek donanım** (Jetson Orin)
tarafı. KCF tracker (C++) + klasik PN teacher (Python) + mavlink-router.

> **Amaç:** Drone düşse / Orin bozulsa, **yeni bir Orin'de** bu repoyu `git clone`
> edip aşağıdaki adımları izleyerek sistemi sıfırdan çalışır hale getirmek.

---

## 1. Donanım

| Parça | Model |
|-------|-------|
| Uçuş kontrol kartı | CUAV V5+ (ArduCopter 4.6.3) |
| Companion bilgisayar | Jetson Orin Nano |
| Yer istasyonu + kamera | SIYI MK15 + entegre FPV kamera (RTSP) |
| Bağlantı | Orin ↔ CUAV: **Type-C/USB** (`/dev/ttyACM0`) · Orin ↔ kamera: **Ethernet** (192.168.144.25) |

## 2. Test Edilmiş Ortam

JetPack 6 (L4T R36.4.7) · Ubuntu 22.04 aarch64 · OpenCV 4.10.0 · gcc 11.4 · cmake 3.22 ·
Python 3.10 · pymavlink 2.4.49 · numpy 1.25 · mavlink-router v4

## 3. Mimari

```
SIYI kamera ──Ethernet/RTSP──► Orin ──(KCF tracker)──► RTSP OUT ──► MK15 ekran
                                  │
CUAV V5+ ──Type-C/USB──► mavlink-router ──┬─UDP 14550─► runtracker (RC oku + OSD)
   (/dev/ttyACM0)        (portun TEK sahibi) ├─UDP 14551─► controller_orin (ATT/RC oku + RC_OVERRIDE yaz)
                                            └─UDP 14552─► QGC/Mission Planner (opsiyonel)

runtracker ──TCP FRAME :9999──► controller_orin (bbox: cx,cy,w,h,ang_x,ang_y,state)
MK15 ──SBUS──► alıcı ──► CUAV   (MANUEL uçuş — Orin'e değmez)
```

**Neden mavlink-router:** `/dev/ttyACM0` seri portunu aynı anda tek process açabilir.
runtracker hem RC hem OSD için, controller_orin hem telemetri hem RC_OVERRIDE için MAVLink
ister → çakışma. Router portu tek başına açar, herkese UDP'den dağıtır.

---

## 4. Sıfırdan Kurulum (yeni Orin)

### Adım 1 — Repoyu al + bağımlılıklar
```bash
git clone <repo-url> hunter-orin
cd hunter-orin
./scripts/setup_orin.sh        # APT, GStreamer, pymavlink/numpy, mavlink-router derler
```
`dialout` grubuna eklendiyse **logout/login** (veya reboot).

> **mavlink/ klasörü:** Repoda `mavlink/` (header-only MAVLink) bulunur. Yoksa eski
> Orin'den `kcfTrackerLib/mavlink/` kopyala ya da
> `git clone https://github.com/mavlink/c_library_v2 mavlink` ile al. CMakeLists bunu
> `../mavlink` olarak bekler (repo kökünde `kcf/` ile yan yana).

### Adım 2 — Kamera ağı
```bash
./scripts/camera_route.sh      # SIYI 192.168.144.25'e route ekler, ping ile test eder
```
USB-Ethernet adaptörünün adı yeni Orin'de farklı olur (`enx...` değişir) — script otomatik
tespit eder. Adaptöre IP yoksa script sana komutu söyler.

### Adım 3 — FC parametreleri (KRİTİK — bir kez, kalıcı yaz)
Mission Planner'ı router'a **ağdan** bağla (TCP, `<Orin-IP>:5760` — IP için `hostname -I`),
Full Parameter List'te şunları ayarla + **Write Params**:

| Parametre | Değer | Neden |
|-----------|-------|-------|
| `RC_OVERRIDE_TIME` | **0.3** | RC_OVERRIDE kesilince FC 0.3s'de manuele döner (güvenlik backstop) |
| `SYSID_MYGCS` | **255** | ArduPilot RC_OVERRIDE'ı sadece bu sysid'den kabul eder; controller_orin = 255 |
| `FS_GCS_ENABLE` | 0 | GCS failsafe karışmasın (bench) |

> ACRO parametreleri (`ACRO_OPTIONS=2`, `ACRO_BAL_*=0`, `RC1-4_DZ=0` vb.) **controller_orin
> `--enable-override` modunda otomatik set eder** — elle gerek yok.

**RC kanal haritası** (MK15, runtracker okur):
- **CH5** > 1700 → tracker başlat · < 1300 → durdur
- **CH6** > 1700 → OTONOM · LOW/MID → manuel *(MID güvenlikli)*
- **CH12** < 1350 → ROI kutusu aç · > 1600 → kapat
- **CH13** → kutu boyutu (< 1330 küçült, > 1650 büyüt)

### Adım 4 — Derle
```bash
cd kcf
mkdir -p build && cd build
cmake ..
make kcfTracker -j4
```

### Adım 5 — Çalıştır (3 terminal)
```bash
# Terminal 1
./scripts/run_router.sh
# Terminal 2 (router açıkken)
./scripts/run_tracker.sh
# Terminal 3 — VARSAYILAN DRY-RUN (motor YOK)
./scripts/run_brain.sh
```

---

## 5. Kademeli Bring-up (PERVANESİZ — güvenlik)

Yeni kurulumda sırayı atlama:

1. **Faz 1 — runtracker tek başına:** `run_router` + `run_tracker`. Kumandada görüntü+OSD,
   `[RC RATE]` akıyor mu, CH12<1350 ile mavi kutu açılıyor mu, CH5 ile tracking (turuncu).
2. **Faz 2 — FRAME:** `run_brain` (dry-run). `[FRAME] bağlandı`, `[FC] autopilot sys=2`.
   FOV doğrula (aşağı).
3. **Faz 3 — DRY-RUN doğrulama:** CH6 HIGH → `[DRY|autonomous]` + ACTION satırı. obs/action
   gerçek sensörle makul mü (`flight_*.csv`). **Drone tepki vermez.**
4. **Faz 4 — RC_OVERRIDE CANLI (PERVANELER ÇIKIK):** `./scripts/run_brain.sh --enable-override`.
   - **Eksen yönü ölç:** hedefi oynat → motorlar doğru yönde mi. `controller_orin.py`'de
     `AUTO_PITCH_DIR/ROLL/YAW/THROTTLE_DIR` sim'den geldi, gerçekte ters olabilir → düzelt.
   - **Manuel dönüş testi:** CH6 LOW → motorlar ANINDA pilota dönmeli (`[SAFE] OVERRIDE KESILDI`).

## 6. FOV Kalibrasyonu (önemli)

`ang_x/ang_y = (cx−0.5)·HFOV`. Varsayılan `HFOV=80°, VFOV=50.534°` (Gazebo). SIYI'nin gerçek
FOV'u farklıysa PN gain'leri kayar. Ölç ve ver:
```bash
RTSP=... ./scripts/run_tracker.sh --hfov <gerçek> --vfov <gerçek>
```

## 7. Güvenlik Notları

- **Bench'te pervaneler her zaman çıkık.** `--enable-override` motorları döndürür.
- Manuele dönüş **iki katmanlı:** (a) controller_orin CH6 LOW'da anında `0` release (release_burst
  5×), (b) `RC_OVERRIDE_TIME=0.3` donanım backstop'u (Python çökse bile).
- Manuel uçuş MK15→alıcı→FC **doğrudan** (SBUS). Orin/override devre dışı kalsa pilot kontrolü kaybetmez.

## 8. Kontrolcü (modüler)

- Varsayılan: `mpc_teacher1.py` (klasik PN, **hedef boyutu varsaymaz**, GPS-free — sadece bbox-delta).
- Değiştir: `./scripts/run_brain.sh --teacher <modul_adi>` (örn. başka bir kontrol dosyası).
- 14 girdi / 4 çıktı arayüz sözleşmesi: **`kcf/CONTROLLER_INTERFACE.md`** (üçüncü şahısa verilebilir).

## 9. Dosya Haritası

```
hunter-orin/
├── kcf/
│   ├── include/            KCF + OSD + shared_data header'lari
│   ├── src/                kcftracker.cpp, fhog.cpp
│   ├── runtracker.cpp      tracker + RTSP + MAVLink(UDP) + TCP FRAME server
│   ├── controller_orin.py  Python beyin: obs → mpc_teacher → RC_OVERRIDE (dry-run/override)
│   ├── mpc_teacher1.py     PN teacher (hedef-boyutsuz, GPS-free)
│   ├── CONTROLLER_INTERFACE.md   14-in/4-out arayuz sozlesmesi
│   └── CMakeLists.txt      (Gazebo'suz)
├── config/mavlink-router/main.conf
├── scripts/                setup_orin · camera_route · run_router · run_tracker · run_brain
├── mavlink/                (header-only MAVLink — repoya dahil/kopyalanir)
└── README.md
```

## 10. Hızlı Tanı

| Belirti | Bak |
|---------|-----|
| `Could not open resource` (RTSP) | `./scripts/camera_route.sh` — kamera route'u yok |
| `port mesgul` (router) | başka process `/dev/ttyACM0`'i tutuyor — kapat |
| RC/ATTITUDE gelmiyor | controller_orin/runtracker stream istiyor; router açık mı |
| OSD modu titriyor | (çözüldü — runtracker sadece autopilot heartbeat'i okur) |
| Manuele dönüş yavaş | `RC_OVERRIDE_TIME=0.3` yazıldı mı + release `0` mı |
| RC_OVERRIDE etkisiz | `SYSID_MYGCS=255` mi (controller_orin 255'ten gönderir) |

## Troubleshooting: MK15'te 3-4 sn video donmasi
Kok neden (10 Haz 2026'da kapatildi): Orin RTL8111 <-> SIYI VTX arasi EEE/802.3az
uyumsuzlugu eno1'de link flap yaratiyor (her flap = 2-4 sn karanlik).
Teshis: `cat /sys/class/net/eno1/carrier_changes` artisi + `journalctl -k | grep eno1`.
Cozum: setup_orin.sh dispatcher kurar; dogrulama: `sudo ethtool --show-eee eno1` -> disabled.
Bitrate tavani ~2500 kbps (SIYI paylasimli bant; 5k+ linki bogar).
