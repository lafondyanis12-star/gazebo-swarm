# Rapport — Communication et élection de leader pour un essaim de drones (Gazebo + PX4)

Ce rapport reprend, dans l'ordre où ça s'est passé, le travail fait pour mettre en place la communication et l'élection de leader d'un essaim de drones sous Gazebo et PX4. Version anglaise dans `RAPPORT_EN.md`.

## Contexte

Le stage porte sur un essaim de 3 drones qui doivent communiquer entre eux, détecter et annoncer la déconnexion d'un des drones, et élire un nouveau leader quand le leader (`drone_0`) tombe en panne — sans intervention humaine.

Le projet avait initialement démarré sous Webots — un choix de départ qui s'est révélé ne pas être le bon. Une première version y avait été écrite (`~/mavic2pro.c`, gardé hors de ce dépôt, à titre de référence) : diffusion de position GPS toutes les 0.2s via `emitter`/`receiver`, détection de timeout à 3s, vol en formation V avec `drone_0` codé en dur comme leader. Après discussion, et en prenant en compte les bases du projet de Yassine, il est apparu plus pertinent de passer sur Gazebo, le simulateur conseillé par le professeur pour ce stage — l'occasion aussi de prendre en main l'outil réellement attendu plutôt que de rester sur un choix personnel de départ.

L'objectif de cette phase était donc de reprendre la logique déjà écrite sous Webots (communication + détection + élection, pas encore le vol en formation) et de la refaire avec les outils réellement utilisés pour le stage : Gazebo Sim et PX4 SITL. Ce qui manquait déjà dans la version Webots, et qui reste le cœur du travail présenté ici, c'était l'élection : si le leader disparaît, personne ne prend sa place, les suiveurs continuent d'essayer de suivre un drone qui n'existe plus.

## Choisir comment les drones vont se parler

Avant de coder quoi que ce soit, il fallait déterminer comment faire communiquer les drones sous Gazebo/PX4, puisqu'il n'existe pas d'équivalent tout fait à l'`emitter`/`receiver` de Webots dans cet environnement. Trois pistes ont été envisagées.

La première était d'écrire un plugin Gazebo Transport (`gz-transport`) qui publierait/écouterait des messages directement dans le simulateur, sans passer par PX4 ni par ROS. C'est conceptuellement le plus proche de ce que faisait Webots, mais ça implique d'écrire et de compiler un plugin C++, ce qui alourdit pas mal la mise en place.

La deuxième était ROS 2 : chaque drone publie sa position sur des topics ROS 2. Le problème, c'est qu'il n'y avait aucun espace de travail ROS 2 sur la machine, donc cette option a été écartée pour cette phase.

La troisième, retenue, consiste à faire tourner PX4 en SITL (Software In The Loop, PX4 tourne sur l'ordinateur au lieu d'un vrai drone) et à lire/piloter chaque instance via MAVSDK, la bibliothèque C++ officielle qui parle MAVLink. PX4-Autopilot était déjà cloné sur la machine, et cette architecture est la plus proche de ce que serait un vrai déploiement multi-drones.

MAVLink et MAVSDK ne servent en fait qu'à récupérer la position réelle de chaque drone — la partie "annoncer sa position aux autres et élire un leader" ne correspond à aucun protocole standard, il a donc fallu en écrire un maison (détaillé plus bas).

### Les protocoles en jeu, et ce qu'ils font

Un protocole, c'est une règle que deux programmes suivent pour se comprendre. Voici, du plus bas niveau au plus haut niveau, ce qui intervient réellement dans ce projet :

- **IP** : la base de toute communication réseau, y compris entre deux programmes sur le même ordinateur (adresse `127.0.0.1`, "localhost"). Jamais manipulé directement, mais tout le reste repose dessus.
- **UDP** : envoie un paquet sans connexion préalable et sans garantie d'arrivée, comme une lettre postée sans accusé de réception. Utilisé à la fois pour MAVLink et pour le protocole swarm maison — adapté ici parce que les messages sont répétés très souvent (5 fois par seconde), donc perdre un paquet de temps en temps n'a pas d'importance.
- **TCP** : garantit l'arrivée des données dans l'ordre, au prix d'une vraie connexion établie au préalable. Pas utilisé directement dans le code, mais Gazebo Transport (voir plus bas) s'en sert en interne.
- **MAVLink** : le format de messages standardisé du monde des drones (position, vitesse, batterie...), que quasiment tous les autopilotes du marché savent lire et écrire. C'est ce que parle PX4 pour annoncer sa position, transporté par-dessus UDP. Le programme (`swarm_node.cpp`) parle directement MAVLink à PX4 via la bibliothèque MAVSDK, sans intermédiaire : la chaîne complète pour lire une position est simplement `swarm_node → MAVLink/UDP → PX4`.
- **Gazebo Transport (gz-transport)** : le système interne de Gazebo, en publish/subscribe — un programme publie sur un canal nommé, un autre s'y abonne. C'est ce qui est derrière toutes les commandes `gz topic`/`gz service` utilisées ici (lister les drones présents, en déplacer un, déclencher un reset).
- **Protocole swarm maison** : détaillé plus bas.

## Mettre PX4 et Gazebo en état de marche

**Prévu :** compiler PX4 pour la version moderne de Gazebo (gz-sim, pas gazebo-classic) et installer MAVSDK, pour pouvoir commencer à coder la logique de communication. Cette étape s'est révélée plus longue que prévu à cause de trois problèmes assez éloignés du sujet du stage, mais qu'il fallait résoudre pour avancer.

**Problème rencontré (1/3) :** la version de Python installée via Homebrew (3.14) était cassée — une bibliothèque interne (`expat`) ne correspondait pas à ce que Python attendait, ce qui rendait `pip` totalement inutilisable.
**Solution :** utiliser partout le Python système d'Apple (`/usr/bin/python3`), qui fonctionnait correctement.

**Problème rencontré (2/3) :** PX4 avait son propre environnement Python isolé (`.venv`), construit sur ce même Python cassé. `pip` n'existait même pas dedans, donc impossible d'installer les dépendances de build comme `kconfiglib`.
**Solution :** reconstruire ce `.venv` avec le Python système, puis réinstaller dedans tout ce que demande `Tools/setup/requirements.txt`.

**Problème rencontré (3/3) :** une fois la compilation relancée, elle échouait sur un plugin Gazebo servant à détecter le mouvement par caméra (`optical_flow`), pas du tout utilisé pour ce qu'on fait ici, mais compilé par défaut. Ce plugin réclame une ancienne API d'OpenCV (`opencv2/core/types_c.h`) supprimée dans la version installée sur la machine (5.0, trop récente).
**Solution :** plutôt que de remplacer OpenCV et risquer de casser autre chose, installer l'ancienne version (`opencv@4`) en parallèle, et reconfigurer CMake pour cibler cette version-là uniquement pour ce module.

Une fois ces trois points réglés, `make px4_sitl` compile sans erreur et produit le binaire `build/px4_sitl_default/bin/px4`.

## Communication et élection de leader

**Prévu :** chaque drone lit sa position réelle, la partage avec les deux autres, détecte une déconnexion (timeout de 3s, comme Webots), et élit un nouveau leader quand le leader disparaît.

Le principe retenu tient en deux briques. La première : chaque drone se connecte en MAVSDK à sa propre instance PX4 et lit sa position GPS réelle, calculée par la physique de Gazebo. La deuxième : chaque drone diffuse cette position aux deux autres, cinq fois par seconde, via un petit maillage UDP écrit pour l'occasion — l'équivalent direct de l'`emitter`/`receiver` de Webots. Chaque drone garde en mémoire l'instant du dernier message reçu de chaque autre drone ; au-delà de 3 secondes sans nouvelle (même seuil que le code Webots), il le considère déconnecté.

L'élection découle d'une règle simple que chaque drone applique de son côté, sans concertation : le leader devrait toujours être le plus petit numéro encore vivant parmi ceux qu'il voit. Comme les trois drones voient à peu près le même réseau, ils arrivent tous à la même conclusion.

### Le format du protocole maison

Chaque message envoyé fait 36 octets :

| Champ | Taille | Contenu |
|---|---|---|
| `sender_id` | 4 octets | l'ID du drone émetteur (0, 1 ou 2) |
| `x`, `y`, `z` | 8 octets chacun | latitude, longitude, altitude |
| `timestamp` | 8 octets | instant d'envoi |

C'est défini dans `swarm_node.cpp` par une struct `SwarmPacket` (`sender_id` en `int32_t`, le reste en `double`, avec `#pragma pack(1)` pour éviter que le compilateur n'ajoute du remplissage entre les champs) — directement inspiré du `Packet` du code Webots, qui faisait la même chose.

### Ce qui n'a pas marché du premier coup

**Problème rencontré (1/3) :** la toute première version faisait connecter chaque drone, en MAVSDK, aux trois instances PX4 à la fois — la sienne et celles des deux autres. Ça semblait plus simple, pas besoin d'inventer un protocole. Sauf qu'en testant, aucune détection ne fonctionnait. La raison : PX4 envoie ses informations vers un port cible fixe (14540+id), et un port ne peut être écouté que par un seul programme à la fois sur la machine. Si `drone_1` et `drone_2` essaient tous les deux d'écouter `drone_0` sur ce même port, un seul y arrive vraiment.
**Solution :** revenir au principe des deux briques décrit plus haut — chaque drone n'écoute que sa propre instance, et redistribue lui-même l'info via le maillage UDP maison.

**Problème rencontré (2/3) :** MAVSDK affichait un avertissement de dépréciation sur l'URL de connexion `udp://`.
**Solution :** remplacée par `udpin://0.0.0.0:PORT` (même comportement, syntaxe plus explicite).

**Problème rencontré (3/3) :** au moment de tester l'élection elle-même, arrêter seulement le programme PX4 du leader ne suffisait pas — son processus de communication continuait de tourner et de rediffuser en boucle sa dernière position connue, donc les autres drones pensaient toujours recevoir de ses nouvelles.
**Solution :** pour simuler une vraie panne, arrêter les deux programmes du drone concerné, PX4 et le processus de communication — comme dans la vraie vie, une panne coupe tout d'un coup, pas juste une partie.

**Problème rencontré (4/4) :** une fois le programme compilé et lancé, plus aucun message ne s'affichait, alors que les logs internes de MAVSDK montraient bien que la connexion à PX4 avait réussi. Le programme n'était pas planté, juste silencieux.
**Solution :** quand la sortie standard (`std::cout`) n'est pas connectée à un vrai terminal (par exemple redirigée vers un fichier), elle est mise en mémoire tampon et ne s'affiche que quand le tampon se remplit ou que le programme se termine. Ajouter `std::cout.setf(std::ios::unitbuf)` en début de programme force un affichage immédiat à chaque ligne.

**Test réalisé :** les trois drones se détectent bien mutuellement, et en tuant `drone_0` (les deux processus), `drone_1` et `drone_2` détectent la perte en environ 3 secondes et élisent tous les deux `drone_1` comme nouveau leader.

## Rendre le monde Gazebo persistant

**Prévu :** faire en sorte qu'un reset de la simulation (le bouton qui remet le chronomètre à zéro) ne fasse pas disparaître les trois drones.

Au départ, les trois drones étaient créés dynamiquement par PX4 à chaque lancement : jamais écrits dans le fichier du monde, juste ajoutés en mémoire. Conséquence, un reset recharge le monde depuis son fichier d'origine, et les drones disparaissaient.

**Solution générale :** les écrire directement dans le fichier du monde (`worlds/swarm_persistent.sdf`), à des positions fixes, puis dire à PX4 de ne plus créer de drone mais de s'attacher à celui qui existe déjà (variable `PX4_GZ_MODEL_NAME` à la place de `PX4_SIM_MODEL`). Le fichier du monde vit dans ce projet, mais PX4 cherche ses mondes dans son propre dossier (`Tools/simulation/gz/worlds/`) — plutôt que de dupliquer le fichier à deux endroits, un lien symbolique a été créé dans le dossier de PX4, pointant vers le vrai fichier conservé ici.

**Problème rencontré :** une fois ça en place, `drone_0` s'attachait bien au monde, mais `drone_1` et `drone_2` restaient bloqués indéfiniment à répéter "en attente de Gazebo". La cause, un peu subtile : le fichier de configuration du modèle `x500` donne par défaut le nom `"default"` au monde Gazebo si rien n'est précisé, et le vrai nom (`swarm_persistent`) n'avait été donné qu'à `drone_0`.
**Solution :** préciser explicitement le nom du monde pour les trois instances, pas seulement la première.

**Test réalisé :** déclenchement d'un reset via le service de contrôle de Gazebo — les trois drones restent bien en place, PX4 continue de tourner normalement.

**Point de vigilance noté :** déclencher un reset manuellement pendant qu'une instance PX4 tourne depuis longtemps peut désynchroniser son horloge interne (le mode "lockstep" avec Gazebo) et la bloquer silencieusement — le processus reste vivant mais ne répond plus. Une instance fraîchement relancée n'a pas ce problème.

## Portée radio simulée et le bug du split-brain

**Prévu :** pouvoir tester une déconnexion en éloignant physiquement un drone, pas seulement en tuant son programme, pour se rapprocher d'un scénario réaliste.

**Solution mise en place :** une portée radio simulée de 6 mètres, calculée à partir de la vraie distance GPS entre deux drones — un message reçu de plus loin est simplement ignoré, comme le ferait une vraie radio qui perd le signal. Dans la foulée, de vraies fenêtres de terminal ont été ouvertes pour chaque drone, pour que les événements de connexion/déconnexion/élection soient visibles en direct au lieu de tourner silencieusement en arrière-plan.

**Test réalisé :** `drone_2` téléporté à 30 mètres via le service `set_pose` de Gazebo, puis ramené à sa position d'origine.

**Problème rencontré :** ce test a révélé un vrai bug. Un drone isolé, qui ne voit plus personne, s'élit logiquement lui-même leader — mais la règle d'élection d'origine ne redonnait le leadership à personne une fois qu'un drone l'avait pris, même si le vrai leader redevenait joignable. Résultat, après reconnexion, deux drones pouvaient rester chacun persuadés d'être le leader : un cas classique de split-brain en systèmes distribués.

**Solution :** faire recalculer en permanence "qui devrait être leader" (toujours le plus petit ID vivant), plutôt que seulement en réaction à la disparition du leader courant.

**Test de validation :** `drone_2`, isolé puis reconnecté, rend bien la main à `drone_0` dès qu'il le revoit.

## Dépôt Git

**Prévu :** garder une trace versionnée du travail. Installation et authentification de `gh` (connexion par navigateur avec un code à usage unique), configuration de Git, puis initialisation du dépôt, premier commit et création d'un dépôt privé sur GitHub. Aucun problème rencontré à cette étape.

## Tester soi-même

Ouvrir 4 fenêtres de terminal.

Dans la première, lancer `~/Documents/my_project/gazebo_swarm/run_swarm.sh` et attendre que Gazebo s'ouvre avec les trois drones posés au sol. Le bouton "reset" de Gazebo peut être testé à ce stade : les drones doivent rester en place.

D'abord, compiler `swarm_node` (une seule fois, ou à chaque modification du code) :
```bash
cd ~/Documents/my_project/gazebo_swarm
cmake -S . -B build
cmake --build build
```

Dans les trois terminaux suivants, lancer respectivement `./build/swarm_node --id 0`, `--id 1` et `--id 2`. Chaque terminal doit afficher un "SUCCES : signal detecte" pour les deux autres drones après quelques secondes.

Pour simuler la panne du leader :
```bash
pkill -f "bin/px4 -i 0"
pkill -f "swarm_node --id 0"
```
Les terminaux 3 et 4 doivent afficher, après environ 3 secondes, la perte de signal puis l'élection de `drone_1`.

Pour simuler une déconnexion par distance, sans tuer aucun programme :
```bash
gz service -s /world/swarm_persistent/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req "name: 'x500_2', position: {x: 30, y: 0, z: 0}"
```
puis la même commande avec `x: 4` pour ramener le drone. (Possible aussi à la souris dans la fenêtre Gazebo, avec l'outil de translation.)

Pour tout arrêter :
```bash
pkill -f "swarm_node"
pkill -f "bin/px4 -i"
pkill -f "gz sim"
```

Les informations de connexion/déconnexion/leader s'affichent dans les fenêtres des trois `swarm_node`, pas dans Gazebo — Gazebo ne fait que la simulation 3D, il ne sait rien du système de communication.

## Prochaines étapes

Reste à porter la logique de vol en formation V de `mavic2pro.c` vers du contrôle "offboard" MAVSDK (en C++, dans la continuité de `swarm_node.cpp`), pour que le leader élu pilote réellement différemment des suiveurs, et pas seulement au niveau logique. Il resterait aussi à illustrer explicitement le cas d'une coupure réseau permanente : deux groupes qui ne se reconnectent jamais gardent chacun leur propre leader indéfiniment, ce qui est le comportement attendu, mais mérite un test dédié.
