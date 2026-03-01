#!/bin/bash
set -euo pipefail

# Build a macOS .app bundle for MicrometerLogger
# Usage: ./scripts/bundle-macos.sh

APP_NAME="MicrometerLogger"
BUNDLE_NAME="${APP_NAME}.app"
BINARY_NAME="${APP_NAME}"
BUNDLE_ID="com.micrometer.logger"
VERSION="0.1.0"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TARGET_DIR="${PROJECT_DIR}/target/release"
BUNDLE_DIR="${PROJECT_DIR}/target/${BUNDLE_NAME}"

echo "Building release binary..."
cargo build --release --manifest-path="${PROJECT_DIR}/Cargo.toml"

if [ ! -f "${TARGET_DIR}/${BINARY_NAME}" ]; then
    echo "Error: Release binary not found at ${TARGET_DIR}/${BINARY_NAME}"
    exit 1
fi

echo "Creating .app bundle..."
rm -rf "${BUNDLE_DIR}"
mkdir -p "${BUNDLE_DIR}/Contents/MacOS"
mkdir -p "${BUNDLE_DIR}/Contents/Resources"

# Copy binary
cp "${TARGET_DIR}/${BINARY_NAME}" "${BUNDLE_DIR}/Contents/MacOS/"

# Copy icon
if [ -f "${PROJECT_DIR}/assets/icon.icns" ]; then
    cp "${PROJECT_DIR}/assets/icon.icns" "${BUNDLE_DIR}/Contents/Resources/icon.icns"
else
    echo "Warning: assets/icon.icns not found, bundle will have no icon"
fi

# Write Info.plist
cat > "${BUNDLE_DIR}/Contents/Info.plist" << PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleName</key>
    <string>${APP_NAME}</string>
    <key>CFBundleDisplayName</key>
    <string>Optical Micrometer Logger</string>
    <key>CFBundleIdentifier</key>
    <string>${BUNDLE_ID}</string>
    <key>CFBundleVersion</key>
    <string>${VERSION}</string>
    <key>CFBundleShortVersionString</key>
    <string>${VERSION}</string>
    <key>CFBundleExecutable</key>
    <string>${BINARY_NAME}</string>
    <key>CFBundleIconFile</key>
    <string>icon</string>
    <key>CFBundlePackageType</key>
    <string>APPL</string>
    <key>NSHighResolutionCapable</key>
    <true/>
    <key>LSMinimumSystemVersion</key>
    <string>10.13</string>
</dict>
</plist>
PLIST

echo "Bundle created at: ${BUNDLE_DIR}"
echo "To install, copy ${BUNDLE_NAME} to /Applications"
