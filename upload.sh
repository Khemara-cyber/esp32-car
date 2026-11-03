#!/bin/bash
set -e

PIO="/Users/macos/.platformio/penv/bin/pio"

echo "=========================================="
echo "    SWARM-X ESP32 Uploader & Monitor      "
echo "=========================================="

find_port() {
  ls /dev/cu.usbserial* /dev/cu.wchusbserial* /dev/cu.SLAB_USBtoUART* /dev/cu.usbmodem* 2>/dev/null | head -n 1 || true
}

PORT=$(find_port)

if [ -z "$PORT" ]; then
  echo ""
  echo "⚠️  No ESP32 USB device detected yet."
  echo ""
  echo "Common reasons:"
  echo " 1. USB cable: Ensure you are using a DATA cable (charge-only cables power the LED but don't transfer data)."
  echo " 2. Adapter: If using a USB-C adapter/hub, try plugging directly or testing another port."
  echo " 3. Drivers: If your board uses CH340 or CP2102, install the macOS driver."
  echo ""
  echo "Waiting for you to connect your ESP32 (press Ctrl+C to stop)..."
  while [ -z "$PORT" ]; do
    sleep 1
    PORT=$(find_port)
  done
  echo " Detected ESP32 at: $PORT"
fi

echo ""
echo "🚀 Uploading firmware to $PORT ..."
"$PIO" run -t upload --upload-port "$PORT"

echo ""
echo " Upload complete! Starting Serial Monitor at 115200 baud..."
echo "Press Ctrl+C to exit monitor."
"$PIO" device monitor -p "$PORT" -b 115200
