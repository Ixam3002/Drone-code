#include <Wire.h>

#define MPU_ADDRESS 0x68 // I2C address of the MPU-6050
#define SSF_GYRO    65.5 // Sensitivity Scale Factor of the gyro

#define X  0     // X axis
#define Y  1     // Y axis
#define Z  2     // Z axis
#define FREQ 250 // Sampling frequency

int acc_raw[3]  = {0,0,0};
int gyro_raw[3] = {0,0,0};
int period;
int temperature = 0;
unsigned long int loop_timer;
unsigned long int now; // Exists just to reduce the calls to micros()
float gyro_angle[3]  = {0,0,0};
float gyro_offset[3] = {0,0,0};

/**
 * Configure la plage de fonctionnement du gyroscope et de l'accéléromètre :
 *  - accéléromètre: +/-8g
 *  - gyroscope: 500°/sec
 *
 * @return void
 */
void setupMpu6050Registers() {
    // Configure power management
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x6B);                    // Request the PWR_MGMT_1 register
    Wire.write(0x00);                    // Apply the desired configuration to the register
    Wire.endTransmission();              // End the transmission

    // Configure the gyro's sensitivity
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1B);                    // Request the GYRO_CONFIG register
    Wire.write(0x08);                    // Apply the desired configuration to the register : ±500°/s
    Wire.endTransmission();              // End the transmission

    // Configure the acceleromter's sensitivity
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1C);                    // Request the ACCEL_CONFIG register
    Wire.write(0x10);                    // Apply the desired configuration to the register : ±8g
    Wire.endTransmission();              // End the transmission

    // Configure low pass filter
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1A);                    // Request the CONFIG register
    Wire.write(0x03);                    // Set Digital Low Pass Filter about ~43Hz
    Wire.endTransmission();              // End the transmission
}

void readSensor() {
    Wire.beginTransmission(MPU_ADDRESS);// Start communicating with the MPU-6050
    Wire.write(0x3B);                   // Send the requested starting register
    Wire.endTransmission();             // End the transmission
    Wire.requestFrom(MPU_ADDRESS,14);   // Request 14 bytes from the MPU-6050

    // Wait until all the bytes are received
    while(Wire.available() < 14);

    acc_raw[X]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[X] variable
    acc_raw[Y]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Y] variable
    acc_raw[Z]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Z] variable
    temperature = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the temperature variable
    gyro_raw[X] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[X] variable
    gyro_raw[Y] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[Y] variable
    gyro_raw[Z] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[Z] variable

    Serial.println(gyro_raw[X]);

}

void setup(){
  Serial.begin(19200);
  setupMpu6050Registers();

}

void loop(){
  readSensor();
  delay(4);

}

