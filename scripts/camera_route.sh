#!/usr/bin/env bash
# ============================================================================
# SIYI kamera (192.168.144.25) icin ag route'u. Her boot/replug'da gerekir.
# USB-Ethernet adaptorunun adi yeni Orin'de FARKLI olabilir (enx... degisir),
# o yuzden otomatik tespit ediyoruz.
# ============================================================================
CAM_IP="192.168.144.25"
GATEWAY="192.168.144.10"

# 192.168.144.x agindaki arayuzu bul (SIYI subnet'i)
IFACE=$(ip -o addr show | awk '/192\.168\.144\./ {print $2; exit}')

if [ -z "$IFACE" ]; then
    echo "!!! 192.168.144.x agindaki arayuz bulunamadi."
    echo "    Mevcut arayuzler:"
    ip -o addr show | awk '{print "      "$2"  "$4}'
    echo ""
    echo "    SIYI USB-Ethernet adaptoru takili mi? IP'si 192.168.144.x mi?"
    echo "    Adaptore IP vermek icin (ornek):"
    echo "      sudo ip addr add 192.168.144.20/24 dev <ARAYUZ>"
    echo "    Sonra bu script'i tekrar calistir."
    exit 1
fi

echo ">>> Arayuz: $IFACE"
if ip route | grep -q "$CAM_IP"; then
    echo ">>> Route zaten var: $(ip route | grep $CAM_IP)"
else
    sudo ip route add "$CAM_IP" via "$GATEWAY" dev "$IFACE"
    echo ">>> Route eklendi: $CAM_IP via $GATEWAY dev $IFACE"
fi

echo ">>> Kamera ping testi..."
if ping -c 2 -W 2 "$CAM_IP" >/dev/null 2>&1; then
    echo "    OK — kamera erisilebilir ($CAM_IP)"
else
    echo "    !!! Kamera ping'e cevap vermiyor. Adaptor IP'sini ve kabloyu kontrol et."
fi
