/*
 * Contrôleur Webots pour un drone du simulateur d'essaim (gazebo_swarm).
 * Rôle dans le projet : ce fichier est le contrôleur C exécuté individuellement
 * par CHAQUE drone dans la simulation Webots. Il ne pilote pas le vol du drone
 * (les moteurs sont volontairement coupés) : son unique but est de démontrer/
 * tester la reconnaissance réseau au sein de l'essaim, c'est-à-dire la capacité
 * de chaque drone à détecter la présence des autres drones via émetteur/récepteur
 * radio Webots.
 *
 * Objectif : Reconnaissance réseau basique.
 * Zéro physique, zéro déplacement, uniquement de la communication pure.
 */

#include <stdio.h>       // printf/sprintf pour les logs et la construction du nom
#include <stdbool.h>     // type bool pour le tableau "reconnu"
#include <webots/robot.h>    // API générique du robot (init, pas de temps, nom, devices)
#include <webots/motor.h>    // API des moteurs (hélices)
#include <webots/emitter.h>  // API d'émission radio (envoi de messages)
#include <webots/receiver.h> // API de réception radio (écoute de messages)

// La structure du message : on envoie juste son ID
// C'est le format binaire échangé entre drones via l'émetteur/récepteur.
typedef struct { int id; } Packet;

// Point d'entrée du contrôleur Webots.
// Chaque instance de drone dans la simulation exécute ce main() indépendamment.
// argc/argv ne sont pas utilisés ici (pas d'arguments de configuration).
int main(int argc, char **argv) {
  wb_robot_init(); // Initialise la connexion du contrôleur avec le superviseur Webots
  int ts = (int)wb_robot_get_basic_time_step(); // Pas de temps de simulation (ms), utilisé pour cadencer la boucle

  // Récupération du nom et de l'ID (0, 1 ou 2)
  // Le nom du robot Webots est supposé du type "drone_X" : le caractère à
  // l'index 6 (le chiffre après "drone_") donne directement l'ID numérique.
  char name[20];
  sprintf(name, "%s", wb_robot_get_name());
  int my_id = (name[6] - '0');

  // On verrouille les moteurs à 0 pour éviter tout mouvement ou bug physique
  // Les 4 hélices (avant gauche/droite, arrière gauche/droite) sont récupérées
  // comme actionneurs puis mises en mode "vitesse infinie autorisée / vitesse
  // cible nulle" afin que le drone reste immobile pendant tout le test.
  WbDeviceTag m[4] = {
    wb_robot_get_device("front left propeller"),
    wb_robot_get_device("front right propeller"),
    wb_robot_get_device("rear left propeller"),
    wb_robot_get_device("rear right propeller")
  };
  for (int i=0; i<4; i++) {
    wb_motor_set_position(m[i], INFINITY);  // mode vitesse (rotation continue autorisée)
    wb_motor_set_velocity(m[i], 0.0);       // mais vitesse cible = 0 => hélices arrêtées
  }

  // Initialisation des antennes (Émetteur / Récepteur)
  // Ce sont les deux capteurs/actionneurs de communication radio du drone :
  // "em" sert à diffuser son propre ID, "re" sert à écouter les ID des autres.
  WbDeviceTag em = wb_robot_get_device("emitter");
  WbDeviceTag re = wb_robot_get_device("receiver");
  wb_receiver_enable(re, ts); // Active le récepteur avec le même pas de temps que la simulation

  // Tableau pour se souvenir des drones déjà reconnus
  // Indexé par ID de drone (0, 1, 2) ; évite de spammer le message de succès.
  bool reconnu[3] = {false, false, false};

  printf("[%s] Allumé. Moteurs coupés. Recherche de signal...\n", name);

  // Boucle principale de simulation : exécutée à chaque pas de temps Webots.
  // wb_robot_step(ts) fait avancer la simulation d'un pas et retourne -1
  // quand Webots demande l'arrêt du contrôleur (fin de simulation).
  while (wb_robot_step(ts) != -1) {

    // 1. ÉMISSION : Le drone dit "Je suis le drone X" en continu
    Packet tx = {my_id};
    wb_emitter_send(em, &tx, sizeof(Packet));

    // 2. RÉCEPTION : Le drone écoute les messages autour de lui
    // On vide la file d'attente du récepteur à chaque pas de temps.
    while (wb_receiver_get_queue_length(re) > 0) {
      Packet *rx = (Packet *)wb_receiver_get_data(re);

      // Si le message vient d'un AUTRE drone, et qu'on ne l'a pas encore reconnu
      if (rx->id != my_id && !reconnu[rx->id]) {
        printf("\033[1;32m[%s] SUCCÈS : J'ai reconnu le drone_%d !\033[0m\n", name, rx->id);

        // On l'enregistre pour ne pas répéter le message à l'infini
        reconnu[rx->id] = true;
      }

      // On passe au message suivant
      wb_receiver_next_packet(re);
    }
  }

  return 0; // Jamais atteint tant que Webots ne demande pas l'arrêt du contrôleur
}