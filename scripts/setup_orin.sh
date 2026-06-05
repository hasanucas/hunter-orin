#!/usr/bin/env bash
# ============================================================================
# HUNTER Orin — sifirdan kurulum (tek seferlik)
# Test edildi: JetPack 6 (L4T R36.4.x), Ubuntu 22.04, aarch64
# ============================================================================
set -e
echo "=========================================="
echo " HUNTER Orin kurulum"
echo "=========================================="

# --- 1) APT bagimliliklari ---
echo ">>> [1/5] APT paketleri..."
sudo apt update
sudo apt install -y \
    build-essential cmake git pkg-config \
    meson ninja-build \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgstrtspserver-1.0-dev \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    python3-pip lsof

# --- 2) OpenCV kontrolu (JetPack ile gelir, KURMA) ---
echo ">>> [2/5] OpenCV kontrolu..."
if pkg-config --exists opencv4; then
    echo "    OpenCV bulundu: $(pkg-config --modversion opencv4)"
else
    echo "    !!! OpenCV bulunamadi. JetPack normalde OpenCV ile gelir."
    echo "    !!! 'sudo apt install nvidia-opencv' veya JetPack SDK Manager ile kur."
fi

# --- 3) Python paketleri ---
echo ">>> [3/5] Python paketleri (pymavlink, numpy)..."
pip3 install --user pymavlink numpy

# --- 4) mavlink-router (kaynaktan derle) ---
echo ">>> [4/5] mavlink-router..."
if command -v mavlink-routerd >/dev/null 2>&1; then
    echo "    mavlink-router zaten kurulu: $(mavlink-routerd --version 2>&1 | head -1)"
else
    MR_DIR="$HOME/mavlink-router"
    if [ ! -d "$MR_DIR" ]; then
        git clone https://github.com/mavlink-router/mavlink-router.git "$MR_DIR"
    fi
    cd "$MR_DIR"
    git submodule update --init --recursive
    meson setup build . --reconfigure || meson setup build .
    ninja -C build
    sudo ninja -C build install
    cd - >/dev/null
    echo "    mavlink-router kuruldu."
fi

# --- 5) mavlink-router config'i yerine koy ---
echo ">>> [5/5] mavlink-router config..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
sudo mkdir -p /etc/mavlink-router
sudo cp "$REPO_ROOT/config/mavlink-router/main.conf" /etc/mavlink-router/main.conf
echo "    /etc/mavlink-router/main.conf yazildi."

# --- dialout grubu (seri port erisimi) ---
if groups | grep -q dialout; then
    echo ">>> dialout grubu: VAR"
else
    sudo usermod -aG dialout "$USER"
    echo ">>> dialout grubuna eklendin — LOGOUT/LOGIN (veya reboot) gerekli!"
fi

echo ""
echo "=========================================="
echo " Kurulum bitti."
echo " Sonraki adimlar (README.md):"
echo "   1) ./scripts/camera_route.sh      (SIYI kamera agi)"
echo "   2) FC parametreleri (README — RC_OVERRIDE_TIME=0.3, SYSID_MYGCS=255)"
echo "   3) cd kcf && mkdir -p build && cd build && cmake .. && make kcfTracker -j4"
echo "   4) ./scripts/run_router.sh  +  run_tracker.sh  +  run_brain.sh"
echo "=========================================="
