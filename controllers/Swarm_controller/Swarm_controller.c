/*
 * Objectif : Reconnaissance réseau basique.
 * Zéro physique, zéro déplacement, uniquement de la communication pure.
 */

#include <stdio.h>
#include <stdbool.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

// La structure du message : on envoie juste son ID
typedef struct { int id; } Packet;

int main(int argc, char **argv) {
  wb_robot_init();
  int ts = (int)wb_robot_get_basic_time_step();

  // Récupération du nom et de l'ID (0, 1 ou 2)
  char name[20]; 
  sprintf(name, "%s", wb_robot_get_name());
  int my_id = (name[6] - '0'); 

  // On verrouille les moteurs à 0 pour éviter tout mouvement ou bug physique
  WbDeviceTag m[4] = {
    wb_robot_get_device("front left propeller"),
    wb_robot_get_device("front right propeller"), 
    wb_robot_get_device("rear left propeller"),
    wb_robot_get_device("rear right propeller")
  };
  for (int i=0; i<4; i++) { 
    wb_motor_set_position(m[i], INFINITY); 
    wb_motor_set_velocity(m[i], 0.0); 
  }

  // Initialisation des antennes (Émetteur / Récepteur)
  WbDeviceTag em = wb_robot_get_device("emitter"); 
  WbDeviceTag re = wb_robot_get_device("receiver"); 
  wb_receiver_enable(re, ts);

  // Tableau pour se souvenir des drones déjà reconnus
  bool reconnu[3] = {false, false, false};

  printf("[%s] Allumé. Moteurs coupés. Recherche de signal...\n", name);

  while (wb_robot_step(ts) != -1) {
    
    // 1. ÉMISSION : Le drone dit "Je suis le drone X" en continu
    Packet tx = {my_id};
    wb_emitter_send(em, &tx, sizeof(Packet));

    // 2. RÉCEPTION : Le drone écoute les messages autour de lui
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
  
  return 0;
}