# Guide de présentation — Essaim de drones (Gazebo + PX4 + ROS 2)

## 1. Le pitch en une phrase

Communication, élection de leader tolérante aux pannes, et **vol en formation réel** (pas des positions simulées) pour 3 drones, sur PX4 SITL + Gazebo + ROS 2.

## 2. Structure suggérée de la présentation

1. **Problème** — comment faire voler un essaim de drones qui continue à fonctionner même si un drone tombe en panne ou sort de portée radio.
2. **Choix du protocole** — comparaison Gazebo Transport brut / ROS 2 / PX4 SITL+MAVLink/MAVSDK ; choix de PX4+MAVSDK car déjà en place et plus proche d'une vraie flotte.
3. **Architecture** — nœuds ROS 2 (`swarm_comm`), message `SwarmStatus`, boucle de contrôle à 10 Hz.
4. **Démo live** (voir section 4 ci-dessous).
5. **Résultats mesurés** (voir section 5).
6. **Limites connues, assumées honnêtement** (voir section 6) — c'est la partie qui impressionne le plus un prof, ne pas la cacher.
7. **Prochaines étapes.**

## 3. Ce qui est solide et démontrable

- Élection de leader **robuste et "sticky"** (vote majoritaire) : détecte une déconnexion par timeout ET par portée radio simulée (10 m), et ne "reprend" pas bêtement un leader qui vient de revenir (bug de split-brain trouvé et corrigé — voir le rapport).
- **Vol réel** via MAVSDK (arm → takeoff → Offboard), formation triangle qui suit la position et le cap *en direct* du leader (loiter circulaire ou trajectoire en ligne).
- **Anti-collision réactif** (séparation ≥ 1 m) et garde-fous de sécurité (géofence, atterrissage d'urgence si Offboard rejeté).
- **Failover en plein vol** validé en direct : tuer le leader déclenche une réélection réelle en ~3 s, sans que les drones tombent.
- **Retour au point de départ (RTL) propre** à la fin d'un vol — ajouté cette semaine, validé en test automatisé et en vol réel.

## 4. Script de démo pour demain

### Option A — Vol réel PX4 + Gazebo (le plus impressionnant, montre la vraie physique)

Terminal 1 :
```bash
cd ~/Documents/my_project/gazebo_swarm
./run_swarm.sh          # SANS HEADLESS=1 ici : on veut la fenêtre 3D pour le prof
```
Attendre ~45s que les 3 instances PX4 soient prêtes (Gazebo affiche les 3 drones posés).

Terminal 2 :
```bash
cd ~/Documents/my_project/gazebo_swarm/ros2_ws
source /opt/ros/jazzy/setup.bash 2>/dev/null || {
  ROSENV=~/micromamba/envs/ros_env
  export PATH="$ROSENV/bin:$PATH"
  source "$ROSENV/setup.bash"
}
source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch swarm_comm swarm_launch.py
```

**⚠️ Piège rencontré hier soir, à anticiper** : l'armement échoue parfois au premier essai (`Arming denied: Resolve system health failures first`) — c'est transitoire (probablement une course entre la connexion MAVSDK et le heartbeat GCS de PX4). Si ça arrive : `pkill -f "swarm_comm/swarm_node"` puis relancer la même commande. Ça a marché du 1er coup une fois sur deux, du 2e coup à chaque fois. **Prévoir ce coup dans le timing, ne pas paniquer devant le prof si ça arrive.**

Terminal 3 — dashboard live :
```bash
cd ~/Documents/my_project/gazebo_swarm/ros2_ws
source /opt/ros/jazzy/setup.bash 2>/dev/null || {
  ROSENV=~/micromamba/envs/ros_env
  export PATH="$ROSENV/bin:$PATH"
  source "$ROSENV/setup.bash"
}
source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 tools/show_swarm_dashboard.py
```
Affiche : position de chaque drone, qui est leader, connecté/pas connecté par paire, distance minimale (risque de collision), et un **récap d'événements en direct** en bas de l'écran (horodaté) — pratique pour commenter en live sans rien noter à part.

**Séquence de démo, une fois les 3 en vol stable (~15-20s après le décollage) :**

1. Laisser voler quelques secondes, montrer le triangle qui suit le leader sur le cercle.
2. **Déconnexion de drone_0** (simuler une perte de portée radio, sans toucher au code) :
   ```bash
   gz service -s /world/swarm_persistent/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
     --req "name: 'x500_0', position: {x: 2, y: -26, z: 3}"
   ```
   → Dans les ~3s, le dashboard affiche `DISCONNECTED` et un nouveau leader élu (drone_1). C'est une vraie détection, pas un affichage scripté.
3. Laisser drone_0 revenir tout seul (sa propre boucle de contrôle le ramène automatiquement — **rien à scripter pour le retour**, c'est le point à souligner : le système gère lui-même la reconnexion). Il rejoint la formation comme **suiveur**, pas comme leader (élection "sticky", comportement voulu, pas un bug).
4. **Atterrissage / retour au point de départ**, une fois la démo terminée :
   ```bash
   for p in $(ps aux | grep "swarm_comm/swarm_node" | grep -v grep | awk '{print $2}'); do kill -SIGINT "$p"; done
   ```
   → RTL réel confirmé par PX4 (`RTL: start return`, `Landing detected`, `Disarmed by landing`).

**Note technique importante** : ne pas essayer de "tenir" drone_0 loin plus de quelques secondes en le téléportant en boucle (ex: toutes les 1s) — ça a déstabilisé le vol la dernière fois (dérive, oscillations). Un seul téléport suffit à déclencher une vraie déconnexion détectée ; si besoin de prolonger, ne re-téléporter que si sa position s'approche de la portée (surveiller, pas forcer en boucle).

### Option B — Plan B si Gazebo pose problème le jour J

Le dashboard et la logique de communication/élection fonctionnent **sans aucun Gazebo/PX4**, en pilotant les positions directement via des paramètres ROS 2 (le même "mode position de secours" que `test_election.py` utilise) :
```bash
cd ~/Documents/my_project/gazebo_swarm/ros2_ws
source /opt/ros/jazzy/setup.bash 2>/dev/null || {
  ROSENV=~/micromamba/envs/ros_env
  export PATH="$ROSENV/bin:$PATH"
  source "$ROSENV/setup.bash"
}
source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch swarm_comm swarm_launch.py     # sans PX4/Gazebo, aucune erreur
python3 tools/simulate_triangle_flight.py  # dans un 2e terminal, rejoue tout le scénario
```
À dire au prof si utilisé : *"Voici la couche communication/élection, indépendante de la simulation physique — utile pour tester sans dépendre de Gazebo."* C'est un vrai point de conception à valoriser (séparation claire entre logique réseau et pilotage physique), pas juste un filet de sécurité.

## 5. Résultats chiffrés à citer

- Sur 6 vols mesurés (headless, mêmes conditions) : violations de séparation (145, 141, 81, 140, 129, 141 — moyenne 129.5, écart-type ≈22).
- Le mode GUI (fenêtre 3D ouverte) dégrade la stabilité des connexions par rapport au mode headless (le rendu graphique fait concurrence aux 3 instances physiques pour le CPU/GPU) — observé aussi hier soir. **Si le prof demande pourquoi utiliser headless pour les mesures**, c'est la réponse.

## 6. Limites connues — à assumer, pas à cacher

C'est la partie qui donne le plus de crédibilité scientifique : montrer une vraie démarche de debug, pas juste "ça marche".

- **Dérive progressive / géofence** : sur 6 vols, la géofence de sécurité (40 m) s'est déclenchée dans 3 cas sur 6 — un atterrissage d'urgence sûr, pas un crash, mais le mécanisme qui cause la dérive (quasi-collisions physiques répétées dans Gazebo) n'est pas totalement résolu, seulement contenu par le filet de sécurité.
- **Bugs trouvés et corrigés un par un, via télémétrie en direct** (pas devinés) : un "leader fantôme" qui votait sans avoir décollé, une oscillation par manque d'amortissement, un plafond de vitesse qui écrasait l'altitude, une répulsion qui poussait vers le sol. Chacun a une preuve concrète (valeurs de télémétrie réelles) dans le rapport.
- **Nouveau ce soir** : le retour au point de départ (RTL) n'existait pas avant — le projet ne savait atterrir qu'en cas de panne. Ajouté et validé (test automatisé `test_formation.py` passe, + validé en vol réel).

## 7. Questions probables du prof — réponses courtes préparées

- **"Pourquoi PX4/MAVSDK et pas juste ROS 2 pur ?"** → PX4-Autopilot était déjà en place sur la machine, et c'est plus proche de comment une vraie flotte serait pilotée (autopilote réel, pas juste une simulation cinématique).
- **"Comment testez-vous sans faire voler à chaque fois ?"** → `test_election.py` (comm/élection, sans PX4, quelques secondes) pour l'itération rapide ; `test_formation.py` (vol réel complet) pour la validation de bout en bout, lancé à la main.
- **"Que se passe-t-il si 2 drones se perdent en même temps ?"** → Cas géré délibérément de façon prudente : si les 2 survivants sont aussi hors de portée l'un de l'autre au même moment, aucun ne s'auto-élit (empêche le split-brain) — ils restent figés jusqu'à reconnexion, plutôt que de risquer une élection incohérente.
- **"Pourquoi deux modes (simulé et vol réel) ?"** → Ils testent des choses différentes, à des vitesses différentes. Le mode simulé (sans Gazebo/PX4) isole la couche communication/élection — rapide, fiable, utilisé pour le développement et les tests automatisés. Le mode vol réel valide avec de la vraie physique — c'est le seul qui peut révéler des bugs qui n'existent qu'en vol (le "leader fantôme", l'oscillation par manque d'amortissement, la dérive d'altitude). Le fait que les deux modes tournent avec le *même* `swarm_node.cpp` montre que la couche réseau/élection est délibérément découplée du pilotage physique — elle ne sait même pas si elle parle à un vrai drone ou à un paramètre ROS 2 fixé à la main.
- **"C'est quoi la prochaine étape ?"** → Distinguer une proximité chronique (répulsion soutenue) d'une simple rencontre brève, pour réduire encore le taux de déclenchement de la géofence.

## 8. Checklist avant de partir demain

- [ ] Tester `./run_swarm.sh` + `ros2 launch swarm_comm swarm_launch.py` une fois **avant** la présentation pour vérifier que l'armement passe (et connaître le geste de relance si besoin).
- [ ] Avoir les 3 terminaux + raccourcis prêts (historique shell ou ce document ouvert).
- [ ] Vérifier que le Mac a assez de batterie/est branché (PX4+Gazebo+3 nœuds ROS consomment pas mal de CPU).
- [ ] Prévoir ~5 min de démo live max ; le reste en explication/slides.
