#!/bin/bash

# Quick Upload Script for ESP32-S3 Stage 1 Project
# Usage: ./quick_upload.sh [ota|usb|test]

case "$1" in
    "ota")
        echo "🚀 Uploading via OTA (WiFi)..."
        echo "📡 Target: 192.168.1.249"
        time pio run -e esp32s3 -t upload
        ;;
    "usb")
        echo "🔌 Uploading via USB..."
        echo "⚡ High speed USB upload"
        time pio run -e esp32s3_usb -t upload
        ;;
    "test")
        echo "🏓 Testing network connectivity..."
        ping -c 3 192.168.1.249
        echo ""
        echo "📊 Network analysis:"
        echo "- Good: <50ms latency, 0% packet loss"
        echo "- Fair: 50-100ms latency, <5% packet loss"
        echo "- Poor: >100ms latency, >5% packet loss"
        ;;
    *)
        echo "🛠️  ESP32-S3 Upload Options:"
        echo ""
        echo "Usage: ./quick_upload.sh [option]"
        echo ""
        echo "Options:"
        echo "  ota   - Upload via WiFi (slower but wireless)"
        echo "  usb   - Upload via USB cable (faster)"
        echo "  test  - Test network connectivity"
        echo ""
        echo "💡 Tips:"
        echo "- Use 'usb' for development (faster)"
        echo "- Use 'ota' for deployed devices"
        echo "- Run 'test' if OTA uploads are slow"
        ;;
esac 