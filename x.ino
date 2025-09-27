#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// ------------------- NRF24L01 Ayarları -------------------
RF24 alici(9, 10);
const uint64_t kanal = 1111; // 5 bayt adres
int command;

// ------------------- Motor Tanımları -------------------
#define MOTOR1_PIN 3
#define MOTOR2_PIN 4
#define MOTOR3_PIN 6
#define MOTOR4_PIN 5

Servo motor1, motor2, motor3, motor4;

// ------------------- MPU6050 Ayarları -------------------
MPU6050 mpu;
int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
float accRoll, accPitch;
float gyroRoll, gyroPitch;
float roll = 0, pitch = 0;

// ------------------- PID Ayarları -------------------
float kp = 3.0, ki = 0.002, kd = 20.0;
float setpointRoll = 0, setpointPitch = 0;
float previousErrorRoll = 0, previousErrorPitch = 0;
float integralRoll = 0, integralPitch = 0;

// ------------------- Zaman Takibi -------------------
unsigned long previousTime = 0;

// ------------------- Motor Hızları -------------------
int baseSpeed = 1100;
int motorSpeedAdjustment = 0;
int motorSpeed1, motorSpeed2, motorSpeed3, motorSpeed4;
const int maxSpeed = 1600;
const int minSpeed = 1000;

bool pidActive = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // MPU6050 Başlat
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 bağlantı hatası!");
    while (1);
  }

  mpu.setXAccelOffset(-1924);
  mpu.setYAccelOffset(-769);
  mpu.setZAccelOffset(5784);
  mpu.setXGyroOffset(8);
  mpu.setYGyroOffset(65);
  mpu.setZGyroOffset(-21);

  // Motorları başlat
  motor1.attach(MOTOR1_PIN);
  motor2.attach(MOTOR2_PIN);
  motor3.attach(MOTOR3_PIN);
  motor4.attach(MOTOR4_PIN);
  setMotorSpeed(minSpeed);

  // NRF24L01 Başlat
  alici.begin();
  alici.openReadingPipe(1,kanal);
  alici.startListening();

  Serial.println("Drone PID Başlatıldı.");
  previousTime = millis();
}

void loop() {
  // Komutları oku
  if (alici.available()) {
    alici.read(&command, sizeof(command));
    Serial.println(command);

    switch (command) {
      case 5: kp += 0.1; break;
      case 6: kp -= 0.1; break;
      case 7: ki += 0.0001; break;
      case 8: ki -= 0.0001; break;
      case 9: kd += 5; break;
      case 10: kd -= 5; break;

      case 1:
        motorSpeedAdjustment += 50;
        motorSpeedAdjustment = constrain(motorSpeedAdjustment, 0, 500);
        pidActive = true;
        Serial.print("Hız artırıldı: ");
        Serial.println(baseSpeed + motorSpeedAdjustment);
        break;

      case 2:
        motorSpeedAdjustment -= 50;
        motorSpeedAdjustment = constrain(motorSpeedAdjustment, 0, 500);
        Serial.print("Hız azaltıldı: ");
        Serial.println(baseSpeed + motorSpeedAdjustment);
        break;

      case 3:
        motorSpeedAdjustment = 0;
        setMotorSpeed(minSpeed);
        pidActive = false;
        Serial.println("Motorlar durduruldu.");
        break;

      case 4:
        emergencyStop();
        break;
    }
  }

  // Zaman hesapla
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  if (dt >= 0.01 && pidActive) {
    previousTime = currentTime;

    // MPU6050 Verilerini Al
    mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

    // İvme tabanlı açı
    accRoll = atan2(accY, accZ) * 180 / PI;
    accPitch = atan2(-accX, accZ) * 180 / PI;

    // Gyro'dan derece/sn
    gyroRoll = gyroX / 131.0;
    gyroPitch = gyroY / 131.0;

    // Complementary Filter
    roll = 0.98 * (roll + gyroRoll * dt) + 0.02 * accRoll;
    pitch = 0.98 * (pitch + gyroPitch * dt) + 0.02 * accPitch;

    // PID hesapla
    float pidRoll = calculatePID(roll, setpointRoll, previousErrorRoll, integralRoll, dt);
    float pidPitch = calculatePID(pitch, setpointPitch, previousErrorPitch, integralPitch, dt);

    // Motorlara uygula
    motorSpeed1 = constrain(baseSpeed + motorSpeedAdjustment + pidRoll + pidPitch, minSpeed, maxSpeed);
    motorSpeed2 = constrain(baseSpeed + motorSpeedAdjustment - pidRoll + pidPitch, minSpeed, maxSpeed);
    motorSpeed3 = constrain(baseSpeed + motorSpeedAdjustment - pidRoll - pidPitch, minSpeed, maxSpeed);
    motorSpeed4 = constrain(baseSpeed + motorSpeedAdjustment + pidRoll - pidPitch, minSpeed, maxSpeed);

    setMotorSpeeds(motorSpeed1, motorSpeed2, motorSpeed3, motorSpeed4);

    // Debug
    Serial.print("Roll: "); Serial.print(roll);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | PIDs => R: "); Serial.print(pidRoll); Serial.print(" P: "); Serial.println(pidPitch);
  }
}

// PID hesaplaması
float calculatePID(float current, float setpoint, float &prevErr, float &integral, float dt) {
  float error = setpoint - current;
  integral += error * dt;
  float derivative = (error - prevErr) / dt;
  prevErr = error;
  return kp * error + ki * integral + kd * derivative;
}

// Motor hızları
void setMotorSpeeds(int s1, int s2, int s3, int s4) {
  motor1.writeMicroseconds(s1);
  motor2.writeMicroseconds(s2);
  motor3.writeMicroseconds(s3);
  motor4.writeMicroseconds(s4);
}

// Tüm motorlara aynı hız
void setMotorSpeed(int speed) {
  motor1.writeMicroseconds(speed);
  motor2.writeMicroseconds(speed);
  motor3.writeMicroseconds(speed);
  motor4.writeMicroseconds(speed);
}

// Acil durdurma
void emergencyStop() {
  setMotorSpeed(minSpeed);
  motorSpeedAdjustment = 0;
  pidActive = false;
  Serial.println("ACİL DURDURMA AKTİF!");
}
