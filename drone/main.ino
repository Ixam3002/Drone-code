#include <Servo.h>
#include <Wire.h>

#include "asservissement.h"
#include "fonctions.h"
#include "MPU6050.h"
#include "conv_joystick2angle.h"


//Definition des esc
Servo escA, escB, escC, escD;
const int ESCA_PIN = 10, ESCB_PIN = 6, ESCC_PIN = 9, ESCD_PIN = 5; // Broche de signal PWM reliée à l'ESC

const float ANGLE_MAX = 10.0; //deg

float kp_pitch = 30.0, ki_pitch = 1.0, kd_pitch = 12.0;
float kp_roll  = 120.0,  ki_roll  = 0.8,  kd_roll  = 30.0;
float kp_alt   = 4.98980340e+01,  ki_alt   = 8.24779880e-03,  kd_alt   = 1.34603797e+00;

bool Stab_Alt = false;
bool ARMED = false;
float Throttle = 0.0;

float POIDS = 0.610; //kg

MpuData mpu;


void setup() {

  Serial.begin(115200);
  
  // Attache des esc au moteur
  escA.attach(ESCA_PIN, 1000, 2000);
  escB.attach(ESCB_PIN, 1000, 2000);
  escC.attach(ESCC_PIN, 1000, 2000);
  escD.attach(ESCD_PIN, 1000, 2000);

  //Initialization MPU
  initMPU();

  Serial.println();
  Serial.println("================================");
  Serial.println("MPU6050 + KALMAN READY");
  Serial.println("================================");

}

void loop() {
  
  // TODO Reception Radio
  float stick_gx, float stick_gy, float stick_dx, float stick_dy;
  bool button_A, bool button_B, bool button_X, bool button_Y;
  bool button_RB;
  
  CommandJoystick cmd = joystick2angle (stick_gx, stick_gy, 
                                        stick_dx, stick_dy,
                                        ANGLE_MAX);
  
  float target_angle[3] = {cmd.roll, cmd.pitch, cmd.yaw};

  if (button_RB){
    if (ARMED){
      ARMED = false;
    }
    else{
      ARMED = true;
    }
  }

  // uint32_t currentMicros = micros();

  if (ARMED){ //drone armed

  //Reading MPU
  MPU_Angles_ACC(mpu);
  float angles[3] = {mpu.roll, mpu.pitch, mpu.yaw};
  float accel[3] = {mpu.linAccX_world, mpu.linAccY_world, mpu.linAccZ_world};
  
  float vitesse_z = 0.0; // c'est à calculer 
  
  Throttle += cmd.throttle;

  // Appel de la fonction
  CommandesMoteurs motors = calcul_correction_PID(angles, vitesse_z,
        Stab_Alt,
        target_angle, // target_angle
        kp_roll, kp_pitch, 0.1, kp_alt,
        ki_roll, ki_pitch, 0.005, ki_alt,
        kd_roll, kd_pitch, 0.02, kd_alt
  );

  // Envoie des commandes aux motors
  escA.writeMicroseconds(borner_valeur(Throttle + motors.f_corr_A));
  escB.writeMicroseconds(borner_valeur(Throttle + motors.f_corr_B));
  escC.writeMicroseconds(borner_valeur(Throttle + motors.f_corr_C));
  escD.writeMicroseconds(borner_valeur(Throttle + motors.f_corr_D));

  } else{ //Drone unarmed
    escA.writeMicroseconds(1000.0);
    escB.writeMicroseconds(1000.0);
    escC.writeMicroseconds(1000.0);
    escD.writeMicroseconds(1000.0);
  }

  // uint32_t endMicros = micros();

  //send in Radio roll and pitch angle 
  Serial.print(mpu.roll);
  Serial.println(mpu.pitch);


  delay(2); // Simulation d'une boucle à 20Hz (delta_t = 2 ms)
}
