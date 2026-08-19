#!/usr/bin/env bash
# Orchestre toute la sequence de demo sans intervention manuelle au bon
# moment : laisse le vol se stabiliser, simule une deconnexion radio de
# drone_0 (teleportation hors portee), laisse la reconnexion se faire toute
# seule (rien a scripter, c'est le point a souligner en demo), puis termine
# proprement par un RTL (retour au point de depart) sur les 3 drones.
#
# Precondition : le vol est deja en cours (Terminal 1 = run_swarm.sh,
# Terminal 2 = ros2 launch swarm_comm swarm_launch.py), triangle stable.
#
# Usage : ./tools/run_demo_sequence.sh
# Reglable via variables d'environnement si besoin :
#   STABILIZE_S=20 RECONNECT_WAIT_S=30 ./tools/run_demo_sequence.sh
set -euo pipefail

STABILIZE_S=${STABILIZE_S:-15}
RECONNECT_WAIT_S=${RECONNECT_WAIT_S:-25}

echo "[demo] Vol en formation -- attente ${STABILIZE_S}s avant la deconnexion simulee..."
sleep "$STABILIZE_S"

echo "[demo] Deconnexion simulee de drone_0 (teleportation hors portee radio)..."
# Appel du service Gazebo set_pose : téléporte instantanément le modèle
# x500_0 aux coordonnées données (x, y, z), simulant une perte de portée
# radio sans avoir à réellement faire voler le drone jusque là-bas
gz service -s /world/swarm_persistent/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req "name: 'x500_0', position: {x: 2, y: -26, z: 3}"

echo "[demo] Attente de la reconnexion automatique (${RECONNECT_WAIT_S}s, rien a faire)..."
sleep "$RECONNECT_WAIT_S"

echo "[demo] Fin de demo -- retour au point de depart (RTL) sur les 3 drones..."
# Récupère les PID des process swarm_node en cours (un par drone)
pids=$(ps aux | grep "swarm_comm/swarm_node" | grep -v grep | awk '{print $2}')
if [[ -z "$pids" ]]; then
  echo "[demo] Aucun swarm_node trouve -- deja arrete ?" >&2
  exit 1
fi
# SIGINT est intercepté par swarm_node.cpp pour déclencher un atterrissage
# RTL propre plutôt qu'un arrêt brutal en plein vol
for p in $pids; do
  kill -SIGINT "$p"
done
echo "[demo] RTL envoye. Suivre l'atterrissage dans le Terminal 1 (Gazebo+PX4) / Terminal 2 (ROS2)."
