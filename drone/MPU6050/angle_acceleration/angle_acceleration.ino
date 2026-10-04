
#include <Wire.h>

// ============================================================
// MPU6050
// ============================================================

#define MPU_ADDR 0x68

// Gyroscope configuré en ±2000 °/s
// Sensibilité = 16.4 LSB/(°/s)
const float GYRO_SCALE = 16.4;

// Accéléromètre configuré en ±2g
const float ACC_SCALE = 16384;

// ============================================================
// CALIBRATION GYRO
// ============================================================

float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

// ============================================================
// DONNEES MPU
// ============================================================

int16_t rawAccX;
int16_t rawAccY;
int16_t rawAccZ;

int16_t rawGyroX;
int16_t rawGyroY;
int16_t rawGyroZ;

float AccX;
float AccY;
float AccZ;

float GyroX;
float GyroY;
float GyroZ;

// ============================================================
// ANGLES ACCELEROMETRE
// ============================================================

float accRoll;
float accPitch;

// ============================================================
// ANGLES KALMAN
// ============================================================

float roll;
float pitch;

// ============================================================
// TIMING
// ============================================================

const uint32_t LOOP_TIME_US = 2000; // 500 Hz

uint32_t previousMicros = 0;

float v0 = 0.0;
float v;


float accBiasX = 0.0;
float accBiasY = 0.0;
float accBiasZ = 0.0;

float GBias = 1;
float G = 9.81;
// ============================================================
// FILTRE KALMAN 1D
// ============================================================

class Kalman {

public:

  // Angle estimé
  float angle = 0.0;

  // Biais estimé du gyro
  float bias = 0.0;

  // Angle estimé corrigé
  float rate = 0.0;

  // Matrice de covariance
  float P[2][2] = {
    {0, 0},
    {0, 0}
  };

  // ----------------------------------------------------------
  // Paramètres du filtre
  // ----------------------------------------------------------

  // Bruit du processus angle
  float Q_angle = 0.001;

  // Bruit du processus biais gyro
  float Q_bias = 0.003;

  // Bruit de mesure accéléromètre
  float R_measure = 0.03;

  // ----------------------------------------------------------
  // Calcul Kalman
  // ----------------------------------------------------------

  float getAngle(float newAngle, float newRate, float dt)
  {
    // ========================================================
    // ETAPE 1 : PREDICTION
    // ========================================================

    // Compensation du biais gyro
    rate = newRate - bias;

    // Intégration du gyro
    angle += dt * rate;

    // Mise à jour de la covariance
    P[0][0] += dt *
               (dt * P[1][1]
               - P[0][1]
               - P[1][0]
               + Q_angle);

    P[0][1] -= dt * P[1][1];

    P[1][0] -= dt * P[1][1];

    P[1][1] += Q_bias * dt;

    // ========================================================
    // ETAPE 2 : CORRECTION
    // ========================================================

    // Innovation
    float S = P[0][0] + R_measure;

    // Gain de Kalman
    float K[2];

    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Erreur entre accéléromètre et estimation gyro
    float y = newAngle - angle;

    // Correction angle
    angle += K[0] * y;

    // Correction du biais gyro
    bias += K[1] * y;

    // Mise à jour covariance
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;

    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
  }

  // ----------------------------------------------------------
  // Réinitialisation du filtre
  // ----------------------------------------------------------

  void setAngle(float newAngle)
  {
    angle = newAngle;
  }

  float getRate()
  {
    return rate;
  }
};


// ============================================================
// FILTRES KALMAN
// ============================================================

Kalman kalmanRoll;
Kalman kalmanPitch;


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  Wire.begin();

  // I2C rapide
  Wire.setClock(400000);
  writeRegister(0x6B, 0x00);

  delay(100);
  writeRegister(0x1B, 0x18);
  writeRegister(0x1C, 0x00);

  writeRegister(0x1A, 0x03);

  delay(100);

  calibrateGyro_Acc();

  readMPU();

  AccX = rawAccX / ACC_SCALE;
  AccY = rawAccY / ACC_SCALE;
  AccZ = rawAccZ / ACC_SCALE;

  accRoll =
    atan2(
      AccY,
      sqrt(AccX * AccX + AccZ * AccZ)
    ) * 180.0 / PI;

  accPitch =
    atan2(
      -AccX,
      sqrt(AccY * AccY + AccZ * AccZ)
    ) * 180.0 / PI;

  // Initialisation Kalman
  kalmanRoll.setAngle(accRoll);
  kalmanPitch.setAngle(accPitch);

  roll = accRoll;
  pitch = accPitch;

  previousMicros = micros();

  Serial.println();
  Serial.println("================================");
  Serial.println("MPU6050 + KALMAN READY");
  Serial.println("================================");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  uint32_t currentMicros = micros();

  // ----------------------------------------------------------
  // Boucle 500 Hz
  // ----------------------------------------------------------

  if ((uint32_t)(currentMicros - previousMicros) < LOOP_TIME_US)
    return;

  float dt =
    (currentMicros - previousMicros) / 1000000.0;

  previousMicros = currentMicros;

  // ----------------------------------------------------------
  // Lecture MPU
  // ----------------------------------------------------------

  readMPU();

  // ----------------------------------------------------------
  // Conversion gyro
  // ----------------------------------------------------------

  GyroX = rawGyroX / GYRO_SCALE;
  GyroY = rawGyroY / GYRO_SCALE;
  GyroZ = rawGyroZ / GYRO_SCALE;

  // Compensation du biais initial
  GyroX -= gyroBiasX;
  GyroY -= gyroBiasY;
  GyroZ -= gyroBiasZ;

  // ----------------------------------------------------------
  // Conversion accélération
  // ----------------------------------------------------------

  AccX = rawAccX / ACC_SCALE;
  AccY = rawAccY / ACC_SCALE;
  AccZ = rawAccZ / ACC_SCALE;

  AccX -= accBiasX;
  AccY -= accBiasY;
  AccZ -= accBiasZ;

  // ----------------------------------------------------------
  // ANGLES ACCELEROMETRE
  // ----------------------------------------------------------

  accRoll =
    atan2(
      AccY,
      sqrt(AccX * AccX + AccZ * AccZ)
    ) * 180.0 / PI;

  accPitch =
    atan2(
      -AccX,
      sqrt(AccY * AccY + AccZ * AccZ)
    ) * 180.0 / PI;

  // ----------------------------------------------------------
  // KALMAN ROLL
  // ----------------------------------------------------------

  roll =
    kalmanRoll.getAngle(
      accRoll,
      GyroX,
      dt
    );

  // ----------------------------------------------------------
  // KALMAN PITCH
  // ----------------------------------------------------------

  pitch =
    kalmanPitch.getAngle(
      accPitch,
      GyroY,
      dt
    );


  static float yaw = 0.0;

  yaw += GyroZ * dt;

  // ----------------------------------------------------------
  // AFFICHAGE
  // ----------------------------------------------------------

  

  GBias = -AccX * sin(pitch* PI / 180.0) 
      +  AccY * sin(roll* PI / 180.0) * cos(pitch* PI / 180.0) 
      +  AccZ * cos(roll* PI / 180.0) * cos(pitch* PI / 180.0);

  // ----------------------------------------------------------
  // GRAVITE ATTENDUE A L'ANGLE COURANT (roll/pitch en degres -> radians)
  // ----------------------------------------------------------

  float rollRad  = roll  * PI / 180.0;
  float pitchRad = pitch * PI / 180.0;

  float gravBodyX = -sin(pitchRad);
  float gravBodyY =  sin(rollRad) * cos(pitchRad);
  float gravBodyZ =  cos(rollRad) * cos(pitchRad);

  // ----------------------------------------------------------
  // ACCELERATION LINEAIRE (sans gravite), repere DRONE, en m/s^2
  // ----------------------------------------------------------

  float linAccX = (AccX - gravBodyX) * G;
  float linAccY = (AccY - gravBodyY) * G;
  float linAccZ = (AccZ - gravBodyZ) * G;

  // ----------------------------------------------------------
  // INTEGRATION -> VITESSE (repere DRONE)
  // ----------------------------------------------------------

  static float vx = 0.0;
  static float vy = 0.0;
  static float vz = 0.0;

  // Rotation repère DRONE -> MONDE (ZYX, sans yaw car pas de magnétomètre)
  float linAccX_world =
      cos(pitchRad) * linAccX
    + sin(rollRad) * sin(pitchRad) * linAccY
    + cos(rollRad) * sin(pitchRad) * linAccZ;

  float linAccY_world =
      cos(rollRad) * linAccY
    - sin(rollRad) * linAccZ;

  float linAccZ_world =
    - sin(pitchRad) * linAccX
    + sin(rollRad) * cos(pitchRad) * linAccY
    + cos(rollRad) * cos(pitchRad) * linAccZ;

  // Intégration en repère MONDE (fixe, cohérent dans le temps)
  vx += linAccX_world * dt;
  vy += linAccY_world * dt;
  vz += linAccZ_world * dt;

  Serial.print("Roll: ");
  Serial.print(roll, 2);

  Serial.print("  Pitch: ");
  Serial.print(pitch, 2);

  Serial.print("  Yaw: ");
  Serial.print(yaw, 2);

  Serial.print("  AccX: ");
  Serial.print(AccX);

  Serial.print("  AccY: ");
  Serial.print(AccY);

  Serial.print("  AccZ: ");
  Serial.print(AccZ);

  Serial.print(" G : ");
  Serial.print(GBias);


  Serial.print(" Vx : ");
  Serial.print(linAccX);

  Serial.print(" Vy : ");
  Serial.print(linAccY);

  Serial.print(" Vz : ");
  Serial.println(linAccZ);
}


// ============================================================
// LECTURE MPU6050
// ============================================================

void readMPU()
{
  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x3B);

  Wire.endTransmission(false);

  // 6 accel + 2 température + 6 gyro
  Wire.requestFrom(MPU_ADDR, 14, true);

  rawAccX = (Wire.read() << 8) | Wire.read();
  rawAccY = (Wire.read() << 8) | Wire.read();
  rawAccZ = (Wire.read() << 8) | Wire.read();

  // Température
  Wire.read();
  Wire.read();

  rawGyroX = (Wire.read() << 8) | Wire.read();
  rawGyroY = (Wire.read() << 8) | Wire.read();
  rawGyroZ = (Wire.read() << 8) | Wire.read();
}


// ============================================================
// CALIBRATION GYRO
// ============================================================
// ============================================================
// CALIBRATION GYRO + ACCELEROMETRE
// ============================================================

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

  Serial.println("Calibration terminee.");
  Serial.println();

  Serial.print("Gyro Bias X = ");
  Serial.print(gyroBiasX, 4);
  Serial.println(" deg/s");

  Serial.print("Gyro Bias Y = ");
  Serial.print(gyroBiasY, 4);
  Serial.println(" deg/s");

  Serial.print("Gyro Bias Z = ");
  Serial.print(gyroBiasZ, 4);
  Serial.println(" deg/s");

  Serial.println();

  Serial.print("Angle calib Roll  = ");
  Serial.print(calibRoll * 180.0 / PI, 2);
  Serial.println(" deg");

  Serial.print("Angle calib Pitch = ");
  Serial.print(calibPitch * 180.0 / PI, 2);
  Serial.println(" deg");

  Serial.println();

  Serial.print("Acc Bias X = ");
  Serial.println(accBiasX, 4);

  Serial.print("Acc Bias Y = ");
  Serial.println(accBiasY, 4);

  Serial.print("Acc Bias Z = ");
  Serial.println(accBiasZ, 4);

  Serial.println();
}


// ============================================================
// ECRITURE REGISTRE
// ============================================================

void writeRegister(byte reg, byte value)
{
  Wire.beginTransmission(MPU_ADDR);

  Wire.write(reg);
  Wire.write(value);

  Wire.endTransmission(true);
}
