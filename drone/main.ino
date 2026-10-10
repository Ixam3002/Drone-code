#include <Servo.h>
#include <Wire.h>
#include <SPI.h>

#include "asservissement.h"
#include "fonctions.h"
#include "MPU6050.h"
#include "conv_joystick2angle.h"

//Interruption Radio
volatile bool NewMessage = false;
volatile byte RadioData[32];

//Definition des esc
Servo escA, escB, escC, escD;
const int ESCA_PIN = 9, ESCB_PIN = 5, ESCC_PIN = 10, ESCD_PIN = 6;

const float ANGLE_MAX = 10.0; //deg

float kp_pitch = 30.0, ki_pitch = 1.0, kd_pitch = 12.0;
float kp_roll  = 120.0,  ki_roll  = 0.8,  kd_roll  = 30.0;
float kp_alt   = 4.98980340e+01,  ki_alt   = 8.24779880e-03,  kd_alt   = 1.34603797e+00;

bool Stab_Alt = false;
bool ARMED = false;
float Throttle = 1435;

float POIDS = 0.610; //kg

float stick_lx = 0; float stick_ly = 0; float stick_rx = 0; float stick_ry = 0;
bool button_A = false; bool button_B = false; bool button_X = false; bool button_Y = false;
bool button_RB = false;

float target_angle[3]; float angles[3]; float accel[3];


MpuData mpu;

struct __attribute__((packed)) Packet {
  uint8_t header;
  float lx, ly, rx, ry;
  uint8_t rb;
  uint8_t checksum;
};


// Function interruption (ISR)
void interruptionRadio() {
  NewMessage = true;
}


void setup() {

  Serial.begin(115200);

  if (!radio.begin()) {
    Serial.println("NRF24L01 non detecte !");
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0, adresse);
  radio.startListening();

  Serial.println("Recepteur pret");

  //attach interruption on the pin D7
  attachInterrupt(digitalPinToInterrupt(7), interruptionRadio, FALLING);

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

  if (NewMessage) {
    Packet p;
    radio.read(&p, sizeof(p));


    Serial.println(p.rb);
    stick_lx = p.lx;
    stick_ly = p.ly;
    stick_rx = p.rx;
    stick_ry = p.ry;
    button_RB = p.button_RB;
    NewMessage = false;
  }
  else{
      stick_lx = 0;
      stick_ly = 0;
      stick_rx = 0;
      stick_ry = 0;
      button_RB = 0;
  }
  CommandJoystick cmd = joystick2angle (stick_lx, stick_ly,
                                        stick_rx, stick_ry,
                                        ANGLE_MAX);

  target_angle[3] = {cmd.roll, cmd.pitch, cmd.yaw};

  // Detection when button RB is pressed
  if (button_RB && !button_RB_prec) {
    ARMED = !ARMED; // State inversion
  }
  button_RB_prec = button_RB; // Save the new state

  // uint32_t currentMicros = micros();

  if (ARMED){ //drone armed

  //Reading MPU
  MPU_Angles_ACC(mpu);
  angles[0] = mpu.roll;
  angles[1] = mpu.pitch;
  angles[2] = mpu.yaw;
  
  accel[0] = mpu.linAccX_world;
  accel[1] = mpu.linAccY_world;
  accel[2] = mpu.linAccZ_world;

  float vitesse_z = 0.0; // TODO : must be calculated

  Throttle += cmd.throttle; //TODO maybe necessary to modify

  // Calcul PID correction
  CommandesMoteurs motors = calcul_correction_PID(angles, vitesse_z,
        Stab_Alt,
        target_angle, // target_angle
        kp_roll, kp_pitch, 0.1, kp_alt,
        ki_roll, ki_pitch, 0.005, ki_alt,
        kd_roll, kd_pitch, 0.02, kd_alt
  );

  // Send values to motors
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

}
