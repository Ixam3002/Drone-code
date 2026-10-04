#include <Wire.h>

#define MPU_ADDR 0x68

// Constantes de conversion (évite d'allouer de la RAM)
#define GYRO_SCALE 16.4f
#define ACC_SCALE  16384.0f
#define G          9.81f
#define LOOP_TIME_US 2000

// Biais réduits au strict nécessaire (24 octets de RAM au total)
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float accBiasX  = 0.0f, accBiasY  = 0.0f, accBiasZ  = 0.0f;

uint32_t previousMicros = 0;


// ============================================================
// FILTRE KALMAN (RAM optimisée)
// ============================================================
class Kalman {
public:
  float angle = 0.0f;
  float bias  = 0.0f;
  float rate  = 0.0f;
  float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

  // Paramètres constants
  const float Q_angle   = 0.001f;
  const float Q_bias    = 0.003f;
  const float R_measure = 0.03f;

  float getAngle(float newAngle, float newRate, float dt) {
    rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S = P[0][0] + R_measure;
    float K[2] = { P[0][0] / S, P[1][0] / S };

    float y = newAngle - angle;
    angle += K[0] * y;
    bias  += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
  }

  void setAngle(float newAngle) { angle = newAngle; }
  float getRate() { return rate; }
};

Kalman kalmanRoll;
Kalman kalmanPitch;

// Structure de données de sortie
struct MpuData {
  float roll;
  float pitch;
  float yaw;
  float linAccX_world;
  float linAccY_world;
  float linAccZ_world;
};

// ============================================================
// FONCTIONS UTILITAIRES I2C
// ============================================================
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

// Lecture directe des données RAW sans variables globales
void readMPU_RAW(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  Wire.read(); Wire.read(); // Ignorer la température

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

// ============================================================
// CALCUL DES ANGLES ET ACCÉLÉRATIONS
// ============================================================
// Passage par référence (&outData) pour économiser la copie en mémoire
bool MPU_Angles_ACC(MpuData &outData) {
  uint32_t currentMicros = micros();

  if ((uint32_t)(currentMicros - previousMicros) < LOOP_TIME_US) {
    return false; // Pas encore l'heure de mettre à jour
  }

  float dt = (currentMicros - previousMicros) * 0.000001f;
  previousMicros = currentMicros;

  int16_t rawAccX, rawAccY, rawAccZ;
  int16_t rawGyroX, rawGyroY, rawGyroZ;
  readMPU_RAW(rawAccX, rawAccY, rawAccZ, rawGyroX, rawGyroY, rawGyroZ);

  // Conversion et suppression du biais (calculs locaux)
  float GyroX = (rawGyroX / GYRO_SCALE) - gyroBiasX;
  float GyroY = (rawGyroY / GYRO_SCALE) - gyroBiasY;
  float GyroZ = (rawGyroZ / GYRO_SCALE) - gyroBiasZ;

  float AccX = (rawAccX / ACC_SCALE) - accBiasX;
  float AccY = (rawAccY / ACC_SCALE) - accBiasY;
  float AccZ = (rawAccZ / ACC_SCALE) - accBiasZ;

  // Angles Accéléromètre
  float accRoll  = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180.0f / PI;
  float accPitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0f / PI;

  // Correction Kalman
  outData.roll  = kalmanRoll.getAngle(accRoll, GyroX, dt);
  outData.pitch = kalmanPitch.getAngle(accPitch, GyroY, dt);

  static float currentYaw = 0.0f;
  currentYaw += GyroZ * dt;
  outData.yaw = currentYaw;

  // Repère monde
  float rollRad  = outData.roll  * PI / 180.0f;
  float pitchRad = outData.pitch * PI / 180.0f;

  float gravBodyX = -sin(pitchRad);
  float gravBodyY =  sin(rollRad) * cos(pitchRad);
  float gravBodyZ =  cos(rollRad) * cos(pitchRad);

  float linAccX = (AccX - gravBodyX) * G;
  float linAccY = (AccY - gravBodyY) * G;
  float linAccZ = (AccZ - gravBodyZ) * G;

  outData.linAccX_world = cos(pitchRad) * linAccX + sin(rollRad) * sin(pitchRad) * linAccY + cos(rollRad) * sin(pitchRad) * linAccZ;
  outData.linAccY_world = cos(rollRad) * linAccY - sin(rollRad) * linAccZ;
  outData.linAccZ_world = -sin(pitchRad) * linAccX + sin(rollRad) * cos(pitchRad) * linAccY + cos(rollRad) * cos(pitchRad) * linAccZ;

  return true; // Données mises à jour
}


void calibrateGyro_Acc()
{
  const int samples = 2000;

  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;

  long sumAX = 0;
  long sumAY = 0;
  long sumAZ = 0;

  Serial.println();
  Serial.println("==============================");
  Serial.println("CALIBRATION GYRO + ACC");
  Serial.println("==============================");
  Serial.println("NE PAS BOUGER LE DRONE !");
  Serial.println();

  delay(2000);

  for (int i = 0; i < samples; i++)
  {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    // 6 accel + 2 température + 6 gyro
    Wire.requestFrom(MPU_ADDR, 14, true);

    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();

    // Température
    Wire.read();
    Wire.read();

    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();

    sumX += gx;
    sumY += gy;
    sumZ += gz;

    sumAX += ax;
    sumAY += ay;
    sumAZ += az;

    delayMicroseconds(500);
  }

  // ----------------------------------------------------------
  // GYRO : conversion en °/s (biais brut = biais gyro, pas de gravité en jeu)
  // ----------------------------------------------------------

  gyroBiasX = (sumX / (float)samples) / GYRO_SCALE;
  gyroBiasY = (sumY / (float)samples) / GYRO_SCALE;
  gyroBiasZ = (sumZ / (float)samples) / GYRO_SCALE;

  // ----------------------------------------------------------
  // ACCELEROMETRE : moyenne brute, en g
  // ----------------------------------------------------------

  float accRawAvgX = (sumAX / (float)samples) / ACC_SCALE;
  float accRawAvgY = (sumAY / (float)samples) / ACC_SCALE;
  float accRawAvgZ = (sumAZ / (float)samples) / ACC_SCALE;

  // ----------------------------------------------------------
  // Angle réellement mesuré pendant la calibration
  // (le drone peut être légèrement incliné, pas forcément à plat)
  // ----------------------------------------------------------

  float calibRoll =
    atan2(
      accRawAvgY,
      sqrt(accRawAvgX * accRawAvgX + accRawAvgZ * accRawAvgZ)
    ); // radians

  float calibPitch =
    atan2(
      -accRawAvgX,
      sqrt(accRawAvgY * accRawAvgY + accRawAvgZ * accRawAvgZ)
    ); // radians

  // ----------------------------------------------------------
  // Gravité attendue (en g) sur chaque axe à cet angle
  // ----------------------------------------------------------

  float gravExpectedX = -sin(calibPitch);
  float gravExpectedY =  sin(calibRoll) * cos(calibPitch);
  float gravExpectedZ =  cos(calibRoll) * cos(calibPitch);

  // ----------------------------------------------------------
  // Vrai biais capteur = mesuré - gravité attendue
  // ----------------------------------------------------------

  accBiasX = accRawAvgX - gravExpectedX;
  accBiasY = accRawAvgY - gravExpectedY;
  accBiasZ = accRawAvgZ - gravExpectedZ;

  // ----------------------------------------------------------
  // Affichage
  // ----------------------------------------------------------

//   Serial.println("Calibration terminee.");
//   Serial.println();

//   Serial.print("Gyro Bias X = ");
//   Serial.print(gyroBiasX, 4);
//   Serial.println(" deg/s");

//   Serial.print("Gyro Bias Y = ");
//   Serial.print(gyroBiasY, 4);
//   Serial.println(" deg/s");

//   Serial.print("Gyro Bias Z = ");
//   Serial.print(gyroBiasZ, 4);
//   Serial.println(" deg/s");

//   Serial.println();

//   Serial.print("Angle calib Roll  = ");
//   Serial.print(calibRoll * 180.0 / PI, 2);
//   Serial.println(" deg");

//   Serial.print("Angle calib Pitch = ");
//   Serial.print(calibPitch * 180.0 / PI, 2);
//   Serial.println(" deg");

//   Serial.println();

//   Serial.print("Acc Bias X = ");
//   Serial.println(accBiasX, 4);

//   Serial.print("Acc Bias Y = ");
//   Serial.println(accBiasY, 4);

//   Serial.print("Acc Bias Z = ");
//   Serial.println(accBiasZ, 4);

//   Serial.println();
}

void initMPU() {
  Wire.begin();
  Wire.setClock(400000);

  writeRegister(0x6B, 0x00);
  delay(100);

  writeRegister(0x1B, 0x18);
  writeRegister(0x1C, 0x00);
  writeRegister(0x1A, 0x03);
  delay(100);

  calibrateGyro_Acc();

  int16_t rawAccX, rawAccY, rawAccZ, gx, gy, gz;
  readMPU_RAW(rawAccX, rawAccY, rawAccZ, gx, gy, gz);

  float initialAccX = (rawAccX / ACC_SCALE) - accBiasX;
  float initialAccY = (rawAccY / ACC_SCALE) - accBiasY;
  float initialAccZ = (rawAccZ / ACC_SCALE) - accBiasZ;

  float initRoll  = atan2(initialAccY, sqrt(initialAccX * initialAccX + initialAccZ * initialAccZ)) * 180.0f / PI;
  float initPitch = atan2(-initialAccX, sqrt(initialAccY * initialAccY + initialAccZ * initialAccZ)) * 180.0f / PI;

  kalmanRoll.setAngle(initRoll);
  kalmanPitch.setAngle(initPitch);
}
