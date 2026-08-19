#!/usr/bin/env bash
# Lancement en une seule commande de la démo pour le professeur : nettoie
# les process laissés par une exécution précédente, puis ouvre les 3
# fenêtres de terminal dans l'ordre (Gazebo/PX4, communication ROS2,
# dashboard en direct). Affiche ensuite la commande à lancer pour
# déclencher la démo de déconnexion/reconnexion une fois la formation
# stabilisée.
set -euo pipefail

DIR="$HOME/Documents/my_project/gazebo_swarm"

# Ouvre $1 (une commande shell) dans une nouvelle fenêtre de terminal
# visible. Essaie les émulateurs de terminal Linux courants l'un après
# l'autre (le premier installé gagne) ; se rabat sur Terminal.app de macOS
# via osascript. Pas de positionnement de fenêtre -- contrairement à une
# disposition Mac+Sidecar connue à l'avance, la configuration d'écran d'un
# poste Linux quelconque n'est pas quelque chose que ce script peut supposer.
open_terminal() {
  local cmd="$1"
  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal -- bash -c "$cmd; exec bash"
  elif command -v x-terminal-emulator >/dev/null 2>&1; then
    x-terminal-emulator -e bash -c "$cmd; exec bash" &
  elif command -v konsole >/dev/null 2>&1; then
    konsole -e bash -c "$cmd; exec bash" &
  elif command -v xterm >/dev/null 2>&1; then
    xterm -hold -e bash -c "$cmd" &
  elif command -v osascript >/dev/null 2>&1; then
    osascript -e "tell application \"Terminal\" to do script \"$cmd\""
  else
    echo "Aucun emulateur de terminal trouve (gnome-terminal/x-terminal-emulator/konsole/xterm) -- lance a la main :" >&2
    echo "  $cmd" >&2
    return 1
  fi
}

# Préfère un ROS2 installé nativement via apt (standard sous Linux) ;
# sinon se rabat sur un environnement RoboStack/micromamba (utilisé sur
# macOS, où ROS2 n'a pas de paquet natif).
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  ROS_ACTIVATE="source /opt/ros/jazzy/setup.bash"
elif [[ -f "$HOME/micromamba/envs/ros_env/setup.bash" ]]; then
  ROS_ACTIVATE="export PATH=\"$HOME/micromamba/envs/ros_env/bin:\$PATH\" && source \"$HOME/micromamba/envs/ros_env/setup.bash\""
else
  echo "Aucun environnement ROS2 Jazzy trouve (ni /opt/ros/jazzy, ni ~/micromamba/envs/ros_env)." >&2
  exit 1
fi

echo "=== Nettoyage d'une eventuelle simulation precedente ==="
pkill -f "swarm_comm/swarm_node" 2>/dev/null || true
pkill -f "ros2 launch swarm_comm" 2>/dev/null || true
pkill -f "swarm_dashboard.py" 2>/dev/null || true
sleep 1
pkill -x px4 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
sleep 2

echo "=== 1/3 : Gazebo + PX4 ==="
open_terminal "cd '$DIR' && ./run_swarm.sh"

echo "Attente ~45s (demarrage PX4/Gazebo -- drone_1/drone_2 ont besoin de ce temps pour"
echo "verrouiller leur position GPS avant que ROS2 essaie de s'y connecter, sinon ils"
echo "timeout et ne decollent jamais -- voir COMMANDES_DEMO.txt)..."
sleep 45

echo "=== Verification que les 3 instances PX4 tournent bien ==="
# PX4/Gazebo plante occasionnellement au demarrage (une instance qui echoue a
# s'attacher a son modele Gazebo, en silence, sans crash bruyant) -- mieux
# vaut le detecter maintenant et relancer tout de suite que de perdre 45s de
# plus a attendre un timeout ROS2 pour le decouvrir.
missing=0
for i in 0 1 2; do
  if ! pgrep -f "bin/px4 -i $i\$" > /dev/null; then
    echo "  ERREUR : px4 -i $i n'est pas lance (plantage au demarrage)."
    missing=1
  fi
done
if [[ "$missing" == "1" ]]; then
  echo ""
  echo "Une ou plusieurs instances PX4 n'ont pas demarre. C'est un plantage"
  echo "aleatoire connu de PX4/Gazebo au demarrage, pas un bug de ce script --"
  echo "relance simplement ce script depuis le debut, ca repart generalement"
  echo "du bon pied."
  exit 1
fi
echo "Les 3 instances PX4 tournent."

echo "=== 2/3 : ROS2 (communication + vol) ==="
open_terminal "cd '$DIR/ros2_ws' && $ROS_ACTIVATE && source install/setup.bash && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && ros2 launch swarm_comm swarm_launch.py"

echo "Attente ~10s (connexion des noeuds a PX4)..."
sleep 10

echo "=== 3/3 : Dashboard live ==="
open_terminal "cd '$DIR/ros2_ws' && $ROS_ACTIVATE && source install/setup.bash && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && python3 tools/show_swarm_dashboard.py"

echo ""
echo "=== Tout est lance ==="
echo ""
echo "Laisser la formation se stabiliser a l'ecran (~15-20s : triangle forme, les 3 drones CONNECTED)."
echo ""
echo "Pour declencher la demo en direct (drone_1 s'eloigne 10s hors de portee radio puis revient) :"
echo ""
echo "  $DIR/trigger_excursion.sh"
echo ""
echo "(La deconnexion reelle n'est pas garantie a 100% du premier coup -- si le"
echo "dashboard/terminal ROS2 ne montre pas de 'Signal lost' apres quelques"
echo "secondes, relancer simplement ce meme script une 2e fois.)"
echo ""
echo "Pour tout arreter en fin de demo :"
echo ""
echo "  pkill -f swarm_comm/swarm_node; pkill -f 'ros2 launch swarm_comm'; pkill -f swarm_dashboard.py; pkill -x px4; pkill -f 'gz sim'"
echo ""
