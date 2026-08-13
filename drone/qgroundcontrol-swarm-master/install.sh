#!/bin/bash
# SwarmDashboard v2 — Automatic installer
# Usage: bash install.sh /path/to/qgroundcontrol

set -e

QGC_PATH="${1:-$HOME/qgroundcontrol}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Installing SwarmDashboard into: $QGC_PATH"

if [ ! -d "$QGC_PATH/src/FlyView" ]; then
    echo "ERROR: $QGC_PATH/src/FlyView not found."
    echo "Make sure you cloned QGroundControl first:"
    echo "  git clone https://github.com/mavlink/qgroundcontrol.git --recursive"
    exit 1
fi

cp "$SCRIPT_DIR/SwarmDashboard.qml"     "$QGC_PATH/src/FlyView/"
cp "$SCRIPT_DIR/CMakeLists.txt"         "$QGC_PATH/src/FlyView/"
cp "$SCRIPT_DIR/FlyViewWidgetLayer.qml" "$QGC_PATH/src/FlyView/"

echo ""
echo "Files installed successfully."
echo ""
echo "Now rebuild QGC:"
echo "  cd $QGC_PATH/build && ninja -j4"
echo ""
echo "Then launch QGC:"
echo "  LIBGL_ALWAYS_SOFTWARE=1 QSG_RENDER_LOOP=basic ./Debug/QGroundControl"
