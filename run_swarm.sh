#!/usr/bin/env bash
# Lance les 3 instances PX4 dans le monde persistant (swarm_persistent.sdf) -
# les drones sont déjà dans le fichier du monde donc un reset Gazebo ne les
# fait plus disparaître.
set -euo pipefail

PX4_DIR="$HOME/PX4-Autopilot"
BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
WORLD_NAME="swarm_persistent"
# Le monde persistant vit dans ce projet ; un lien symbolique dans
# PX4-Autopilot/Tools/simulation/gz/worlds/ permet à PX4 de le trouver
# (son script de démarrage écrase PX4_GZ_WORLDS via gz_env.sh, donc on ne
# peut pas juste pointer ailleurs par variable d'environnement).
WORLD_LINK="$PX4_DIR/Tools/simulation/gz/worlds/$WORLD_NAME.sdf"
if [[ ! -e "$WORLD_LINK" ]]; then
  ln -sf "$HOME/Documents/my_project/gazebo_swarm/worlds/$WORLD_NAME.sdf" "$WORLD_LINK"
fi

if [[ ! -x "$BIN" ]]; then
  echo "Binaire PX4 introuvable ($BIN). Lance d'abord: cd $PX4_DIR && make px4_sitl" >&2
  exit 1
fi

cd "$PX4_DIR"

pids=()
cleanup() {
  echo "Arrêt des instances PX4..."
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

echo "Démarrage de drone_0 (démarre Gazebo avec le monde persistant, s'attache à x500_0)..."
PX4_GZ_WORLD="$WORLD_NAME" PX4_GZ_MODEL_NAME=x500_0 \
  PX4_SYS_AUTOSTART=4001 "$BIN" -i 0 &
pids+=($!)

sleep 15  # laisse le temps à Gazebo de démarrer avant d'attacher les suivantes

echo "Démarrage de drone_1 (s'attache à x500_1, déjà présent dans le monde)..."
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$WORLD_NAME" PX4_GZ_MODEL_NAME=x500_1 PX4_SYS_AUTOSTART=4001 "$BIN" -i 1 &
pids+=($!)

echo "Démarrage de drone_2 (s'attache à x500_2, déjà présent dans le monde)..."
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="$WORLD_NAME" PX4_GZ_MODEL_NAME=x500_2 PX4_SYS_AUTOSTART=4001 "$BIN" -i 2 &
pids+=($!)

echo "3 drones lancés. Ctrl+C pour tout arrêter."
wait
