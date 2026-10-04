#include <Servo.h>

#include "asservissement.h"
#include "fonctions.h"


//Definition des esc
Servo escA, escB, escC, escD;
const int ESCA_PIN = 10, ESCB_PIN = 6, ESCC_PIN = 9, ESCD_PIN = 5; // Broche de signal PWM reliée à l'ESC


float kp_pitch = 1.49688032e+03, ki_pitch = 1.00000000e-02, kd_pitch = 4.68476324e+01;
float kp_roll  = 2.5000000e+03,  ki_roll  = 1.0000000e-02,  kd_roll  = 9.9814911e+01;
float kp_alt   = 4.98980340e+01,  ki_alt   = 8.24779880e-03,  kd_alt   = 1.34603797e+00;

bool Stab_Alt = true;

float POIDS = 0.550; //kg


void setup() {

  // Attache des esc au moteur
  escA.attach(ESCA_PIN, 1000, 2000);
  escB.attach(ESCB_PIN, 1000, 2000);
  escC.attach(ESCC_PIN, 1000, 2000);
  escD.attach(ESCD_PIN, 1000, 2000);

}

void loop() {

  // Definir les commandes de la manette -> Throttle, angle avec les joysticks

  float esc_throttle;


  float angles[3] = {1.2, -0.5, 0.1}; // Roll, Pitch, Yaw -> vient de la manette


  float vitesse_z = 0.05; // c'est à calculer


  // Appel de la fonction
  CommandesMoteurs moteurs = calcul_correction_PID(angles, vitesse_z,
        Stab_Alt,
        (const float[]){0.0, 0.0, 0.0}, // angle_cible
        kp_roll, kp_pitch, 0.1, kp_alt,
        ki_roll, ki_pitch, 0.005, ki_alt,
        kd_roll, kd_pitch, 0.02, kd_alt

  );

  // Envoie des commandes aux moteurs
  escA.writeMicroseconds(borner_valeur(esc_throttle + moteurs.f_corr_A));
  escB.writeMicroseconds(borner_valeur(esc_throttle + moteurs.f_corr_B));
  escC.writeMicroseconds(borner_valeur(esc_throttle + moteurs.f_corr_C));
  escD.writeMicroseconds(borner_valeur(esc_throttle + moteurs.f_corr_D));



  delay(50); // Simulation d'une boucle à 20Hz (delta_t = 0.05s)
}
