#!/bin/bash
# SwarmDashboard v2 — Installateur automatique.
# Copie les fichiers du widget SwarmDashboard (panneau QML custom affichant
# l'état de l'essaim) dans les sources d'une copie locale de
# QGroundControl, pour qu'il soit compilé avec QGC lors du prochain build.
# Usage : bash install.sh /chemin/vers/qgroundcontrol

set -e

# Répertoire d'installation de QGroundControl : 1er argument, sinon
# ~/qgroundcontrol par défaut
QGC_PATH="${1:-$HOME/qgroundcontrol}"
# Répertoire contenant ce script (là où se trouvent les fichiers à copier),
# quel que soit l'endroit d'où le script est appelé
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Installing SwarmDashboard into: $QGC_PATH"

# Vérifie que le chemin donné est bien une copie de QGroundControl valide
# (présence du dossier src/FlyView, où vit la vue de vol principale)
if [ ! -d "$QGC_PATH/src/FlyView" ]; then
    echo "ERROR: $QGC_PATH/src/FlyView not found."
    echo "Make sure you cloned QGroundControl first:"
    echo "  git clone https://github.com/mavlink/qgroundcontrol.git --recursive"
    exit 1
fi

# Copie le widget QML, sa déclaration CMake et la couche qui l'intègre à la
# vue de vol de QGC (ces 3 fichiers remplacent/complètent ceux de QGC)
cp "$SCRIPT_DIR/SwarmDashboard.qml"     "$QGC_PATH/src/FlyView/"
cp "$SCRIPT_DIR/CMakeLists.txt"         "$QGC_PATH/src/FlyView/"
cp "$SCRIPT_DIR/FlyViewWidgetLayer.qml" "$QGC_PATH/src/FlyView/"

echo ""
echo "Files installed successfully."
echo ""
# Rappel : les fichiers copiés ne sont pas encore compilés, il faut
# recompiler QGC pour que le widget apparaisse dans l'interface
echo "Now rebuild QGC:"
echo "  cd $QGC_PATH/build && ninja -j4"
echo ""
# LIBGL_ALWAYS_SOFTWARE=1 force le rendu logiciel (utile si le GPU/driver
# pose problème) ; QSG_RENDER_LOOP=basic évite certains bugs d'affichage Qt
echo "Then launch QGC:"
echo "  LIBGL_ALWAYS_SOFTWARE=1 QSG_RENDER_LOOP=basic ./Debug/QGroundControl"
