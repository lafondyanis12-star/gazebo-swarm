#!/usr/bin/env python3
"""
SwarmDashboard Synthetic Camera Streamer

Rôle dans le projet gazebo_swarm : ce script simule un flux vidéo caméra pour
chaque drone de l'essaim (car les drones simulés n'ont pas de vraie caméra
utilisable directement par QGroundControl). Il génère une image de type HUD
(head-up display) par drone, incrustée avec la télémétrie réelle (position,
altitude) récupérée depuis Gazebo, puis encode cette image en H.264 et la
diffuse en flux RTP/UDP vers QGroundControl (module VideoReceiver), un flux
UDP distinct par drone.

Generates a synthetic HUD frame per drone showing real telemetry
and streams it as H.264 RTP UDP to QGC VideoReceiver.
Drone 1 -> port 5600
Drone 2 -> port 5601
Drone 3 -> port 5602
"""

import sys
import time
import threading
import math

# Bibliothèque cliente Gazebo Transport (v13) : permet de s'abonner aux
# topics publiés par le simulateur Gazebo (ex : positions des robots).
from gz.transport13 import Node
# Message protobuf Gazebo contenant la liste des poses (position/orientation)
# de tous les objets du monde simulé.
from gz.msgs10.pose_v_pb2 import Pose_V

# GStreamer (via PyGObject/gi) : moteur utilisé pour encoder les images en
# H.264 et les envoyer en flux réseau (RTP sur UDP).
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Numpy et Pillow servent à construire l'image du HUD (dessin 2D) et à la
# convertir en tableau d'octets brut consommable par GStreamer.
try:
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    HAS_PIL = True
except ImportError:
    # Installation automatique si les paquets sont absents de l'environnement.
    HAS_PIL = False
    print("Installing Pillow and numpy...")
    import subprocess
    subprocess.run([sys.executable, "-m", "pip", "install", "Pillow", "numpy", "--break-system-packages"])
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont

Gst.init(None)  # Initialisation obligatoire du framework GStreamer avant toute utilisation

# Paramètres de l'image/flux vidéo généré (résolution et cadence)
WIDTH  = 640
HEIGHT = 480
FPS    = 15

# Déclaration des 3 drones de l'essaim : identifiant logique, nom du modèle
# tel que publié par Gazebo (pour faire correspondre la télémétrie), port UDP
# de diffusion du flux vidéo, et couleur utilisée pour le HUD de ce drone.
DRONES = [
    {"id": 1, "model": "x500_0", "port": 5600, "color": (0, 200, 100)},
    {"id": 2, "model": "x500_1", "port": 5601, "color": (0, 150, 255)},
    {"id": 3, "model": "x500_2", "port": 5602, "color": (255, 180, 0)},
]

# Adresse IP de destination des flux UDP (machine où tourne QGroundControl)
HOST = "172.27.80.1"

# Shared telemetry state
# État partagé (entre le thread d'abonnement Gazebo et les threads de streaming)
# contenant la dernière position connue et le compteur de frames de chaque drone.
telemetry = {d["id"]: {"x": 0.0, "y": 0.0, "z": 0.0, "frame": 0} for d in DRONES}

# Dictionnaires globaux associant, par ID de drone, son pipeline GStreamer
# et son élément "appsrc" (point d'injection des images dans le pipeline).
pipelines = {}
appsrcs   = {}

def create_pipeline(drone_id, port):
    """Construit et démarre le pipeline GStreamer d'un drone.

    Chaîne de traitement (protocole réseau utilisé) :
      appsrc      -> point d'entrée où le script injecte des images brutes RGB
      videoconvert -> conversion de format de pixel si nécessaire
      x264enc     -> encodage vidéo H.264 (latence minimale, débit ~800 kbps)
      rtph264pay  -> empaquetage du flux H.264 en paquets RTP
      udpsink     -> envoi des paquets RTP en UDP vers HOST:port

    Chaque drone dispose de son propre pipeline et de son propre port UDP,
    ce qui permet à QGroundControl de recevoir un flux distinct par drone.
    """
    pipeline_str = (
        f"appsrc name=src_{drone_id} is-live=true block=false format=time "
        f"caps=video/x-raw,format=RGB,width={WIDTH},height={HEIGHT},framerate={FPS}/1 ! "
        f"videoconvert ! "
        f"x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast key-int-max=15 ! "
        f"rtph264pay config-interval=1 pt=96 ! "
        f"udpsink host={HOST} port={port} sync=false"
    )
    pipeline = Gst.parse_launch(pipeline_str)  # Construit le pipeline à partir de la description texte ci-dessus
    appsrc   = pipeline.get_by_name(f"src_{drone_id}")  # Référence vers l'élément appsrc pour y pousser les images
    pipeline.set_state(Gst.State.PLAYING)  # Démarre le pipeline (encodage + envoi réseau actifs)
    print(f"[Drone {drone_id}] Pipeline -> udp://{HOST}:{port}")
    return pipeline, appsrc

def draw_frame(drone_id, color):
    """Dessine une image (frame) synthétique de type HUD pour un drone donné.

    Construit une image RGB en mémoire (ciel, sol, ligne d'horizon, grille,
    réticule, bandeaux d'informations) puis incruste la télémétrie courante
    (altitude, position X/Y, numéro de frame). Retourne les octets bruts de
    l'image, au format attendu par le pipeline GStreamer (caps RGB déclarées
    dans create_pipeline).
    """
    t   = telemetry[drone_id]
    img = Image.new("RGB", (WIDTH, HEIGHT), (10, 10, 20))
    d   = ImageDraw.Draw(img)

    # Sky gradient
    # Dégradé du "ciel" sur la moitié supérieure de l'image
    for y in range(HEIGHT // 2):
        ratio = y / (HEIGHT // 2)
        r = int(10 + ratio * 20)
        g = int(10 + ratio * 30)
        b = int(20 + ratio * 60)
        d.line([(0, y), (WIDTH, y)], fill=(r, g, b))

    # Ground
    # Dégradé du "sol" sur la moitié inférieure de l'image
    for y in range(HEIGHT // 2, HEIGHT):
        ratio = (y - HEIGHT // 2) / (HEIGHT // 2)
        r = int(30 + ratio * 20)
        g = int(20 + ratio * 10)
        b = int(10)
        d.line([(0, y), (WIDTH, y)], fill=(r, g, b))

    # Horizon line
    # Ligne d'horizon séparant ciel et sol
    d.line([(0, HEIGHT // 2), (WIDTH, HEIGHT // 2)], fill=(100, 100, 100), width=1)

    # Grid lines on ground
    # Grille de perspective simulant un sol vu depuis le drone
    for i in range(0, WIDTH, 80):
        d.line([(i, HEIGHT // 2), (WIDTH // 2, HEIGHT)], fill=(50, 50, 50), width=1)
    for j in range(HEIGHT // 2, HEIGHT, 40):
        d.line([(0, j), (WIDTH, j)], fill=(40, 40, 40), width=1)

    # Crosshair
    # Réticule central façon viseur caméra
    cx, cy = WIDTH // 2, HEIGHT // 2
    d.line([(cx - 30, cy), (cx - 8, cy)], fill=color, width=2)
    d.line([(cx + 8,  cy), (cx + 30, cy)], fill=color, width=2)
    d.line([(cx, cy - 30), (cx, cy - 8)], fill=color, width=2)
    d.line([(cx, cy + 8),  (cx, cy + 30)], fill=color, width=2)
    d.ellipse([(cx-4, cy-4), (cx+4, cy+4)], outline=color, width=1)

    # Top bar
    # Bandeau supérieur : identifiant du drone, titre du tableau de bord, FPS
    d.rectangle([(0, 0), (WIDTH, 32)], fill=(0, 0, 0, 180))
    d.text((8, 8),  f"DRONE {drone_id}", fill=color)
    d.text((WIDTH // 2 - 40, 8), f"SWARM DASHBOARD", fill=(200, 200, 200))
    d.text((WIDTH - 100, 8), f"FPS {FPS}", fill=(150, 150, 150))

    # Bottom HUD bar
    # Bandeau inférieur regroupant les informations de télémétrie
    d.rectangle([(0, HEIGHT - 60), (WIDTH, HEIGHT)], fill=(0, 0, 0))

    # Altitude
    # Altitude affichée en valeur absolue de la coordonnée Z (mètres)
    alt = abs(t["z"])
    d.text((10, HEIGHT - 55), "ALT", fill=(150, 150, 150))
    d.text((10, HEIGHT - 36), f"{alt:.1f} m", fill=color)

    # Position
    # Position horizontale X/Y issue de la télémétrie Gazebo
    d.text((WIDTH // 2 - 60, HEIGHT - 55), "POSITION", fill=(150, 150, 150))
    d.text((WIDTH // 2 - 70, HEIGHT - 36), f"X:{t['x']:.1f} Y:{t['y']:.1f}", fill=(200, 200, 200))

    # Frame counter
    # Compteur d'images générées depuis le démarrage du flux, pour ce drone
    d.text((WIDTH - 80, HEIGHT - 55), "FRAME", fill=(150, 150, 150))
    d.text((WIDTH - 80, HEIGHT - 36), f"{t['frame']}", fill=(150, 150, 150))

    # Animated pulse on drone ID
    # Petit point clignotant (pulsation sinusoïdale) indiquant que le flux est actif
    pulse = int(127 + 127 * math.sin(time.time() * 3))
    d.ellipse([(8, HEIGHT - 90), (18, HEIGHT - 80)], fill=(0, pulse, 0))

    # Border
    # Bordure colorée reprenant la couleur associée au drone
    d.rectangle([(0, 0), (WIDTH-1, HEIGHT-1)], outline=color, width=2)

    # Conversion de l'image PIL en tableau numpy puis en octets bruts RGB,
    # format attendu par l'appsrc du pipeline GStreamer.
    return np.array(img).tobytes()

def stream_drone(drone_id, color):
    """Boucle de streaming exécutée dans un thread dédié par drone.

    Génère en continu une frame HUD (draw_frame), l'encapsule dans un
    Gst.Buffer avec un timestamp de présentation (pts) et une durée
    cohérents avec le FPS cible, puis la pousse dans l'appsrc du pipeline
    (déclenchant encodage H.264 + envoi RTP/UDP). Tourne indéfiniment tant
    que le processus est actif ; les erreurs sont journalisées puis la
    boucle continue après une courte pause.
    """
    appsrc = appsrcs[drone_id]
    frame_duration_ns = int(1e9 / FPS)  # Durée d'une frame en nanosecondes, pour l'horodatage GStreamer
    pts = 0  # Presentation timestamp cumulatif injecté dans chaque buffer
    while True:
        try:
            raw = draw_frame(drone_id, color)
            buf = Gst.Buffer.new_wrapped(raw)  # Encapsule les octets bruts dans un buffer GStreamer
            buf.pts      = pts
            buf.duration = frame_duration_ns
            appsrc.emit("push-buffer", buf)  # Injecte le buffer dans le pipeline (déclenche l'encodage/envoi)
            pts += frame_duration_ns
            telemetry[drone_id]["frame"] += 1
            if telemetry[drone_id]["frame"] % (FPS * 5) == 1:
                print(f"[Drone {drone_id}] Streaming frame {telemetry[drone_id]['frame']}")
            time.sleep(1.0 / FPS)  # Cadence la boucle sur le FPS cible
        except Exception as e:
            print(f"[Drone {drone_id}] Stream error: {e}")
            time.sleep(0.1)

def setup_telemetry(node):
    """Subscribe to Gazebo pose topic to get real drone positions.

    S'abonne (protocole Gazebo Transport, pub/sub) au topic
    "/world/default/pose/info" qui diffuse en continu les poses de tous les
    objets du monde simulé. Le callback filtre les poses correspondant aux
    modèles de drones connus (voir DRONES) et met à jour le dictionnaire
    partagé "telemetry" avec leur position X/Y/Z courante.
    """
    def cb(msg):
        # Callback appelé par Gazebo Transport à chaque nouveau message de poses
        for pose in msg.pose:
            for d in DRONES:
                if d["model"] in pose.name:
                    tid = d["id"]
                    telemetry[tid]["x"] = pose.position.x
                    telemetry[tid]["y"] = pose.position.y
                    telemetry[tid]["z"] = pose.position.z
    node.subscribe(Pose_V, "/world/default/pose/info", cb)
    print("Telemetry subscribed: /world/default/pose/info")

def main():
    """Point d'entrée du script : initialise pipelines, abonnement télémétrie
    et threads de streaming, puis lance la boucle d'événements GLib qui fait
    tourner GStreamer jusqu'à interruption (Ctrl+C)."""
    print("SwarmDashboard Synthetic Camera Streamer")
    print("=" * 45)
    print(f"Resolution: {WIDTH}x{HEIGHT} @ {FPS} fps")
    print(f"Codec: H.264 (x264enc zerolatency)")
    print()

    # Create GStreamer pipelines
    # Un pipeline GStreamer + un port UDP par drone
    for d in DRONES:
        pipeline, appsrc = create_pipeline(d["id"], d["port"])
        pipelines[d["id"]] = pipeline
        appsrcs[d["id"]]   = appsrc

    # Subscribe to real pose data
    # Connexion au bus Gazebo Transport pour récupérer les positions réelles
    node = Node()
    setup_telemetry(node)

    # Start streaming threads
    # Un thread daemon par drone, chacun exécutant sa propre boucle de streaming
    for d in DRONES:
        t = threading.Thread(
            target=stream_drone,
            args=(d["id"], d["color"]),
            daemon=True
        )
        t.start()
        print(f"[Drone {d['id']}] Streaming thread started")

    print()
    print("QGC Video Settings:")
    print("  Settings > Video > Source: UDP h.264 Video Stream")
    print("  UDP URL: 0.0.0.0:5600  (for Drone 1)")
    print("  Change port to 5601 or 5602 for other drones")
    print()
    print("Streaming... Press Ctrl+C to stop.")

    # Boucle d'événements principale GLib requise par GStreamer pour fonctionner ;
    # bloque le thread principal tant qu'aucune interruption n'est reçue.
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        # Arrêt propre : coupe tous les pipelines GStreamer avant de quitter
        print("\nStopping all pipelines...")
        for p in pipelines.values():
            p.set_state(Gst.State.NULL)
        sys.exit(0)

if __name__ == "__main__":
    main()
