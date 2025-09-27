#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 mpu;

// Ham sensör verileri
int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;

// Jiroskop kalibrasyon/ofset değerleri
float gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;

// Açı hesaplaması için değişkenler
float angle_pitch = 0, angle_roll = 0;
float accel_angle_pitch = 0, accel_angle_roll = 0;
unsigned long loop_timer;
float dt;

// --- AYARLANACAK PID KATSAYILARI ---
// Bu değerleri Seri Monitör'den değiştireceğiz
float kp_roll = 0.0;
float ki_roll = 0.0;
float kd_roll = 0.0;

// PID hesaplama değişkenleri
float error_roll, prev_error_roll = 0, integral_roll = 0, derivative_roll, pid_output_roll;

// Motorlar için sabit, düşük bir gaz değeri (sadece tepkiyi görmek için)
int tuning_throttle = 1200; 
bool motors_active = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    while (1) Serial.println("MPU6050 bağlantı hatası!");
  }
  
  // Jiroskop kalibrasyonu
  for (int i = 0; i < 2000; i++) {
    mpu.getRotation(&gyroX, &gyroY, &gyroZ);
    gyro_x_cal += gyroX;
    gyro_y_cal += gyroY;
    delay(1);
  }
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;

  // Kullanıcıya talimatları yazdır
  Serial.println("PID Ayarlama Arayüzü Başlatıldı.");
  Serial.println("--------------------------------");
  Serial.println("Komutlar (sonra Enter'a basın):");
  Serial.println("p[deger] -> Kp değerini ayarla (örn: p1.25)");
  Serial.println("i[deger] -> Ki değerini ayarla (örn: i0.05)");
  Serial.println("d[deger] -> Kd değerini ayarla (örn: d0.8)");
  Serial.println("start    -> Motorları test gücünde çalıştır");
  Serial.println("stop     -> Motorları durdur");
  Serial.println("--------------------------------");

  loop_timer = micros();
}

void loop() {
  // Gelen seri komutları kontrol et
  if (Serial.available() > 0) {
    parseSerialCommand();
  }

  // Zaman farkını hesapla
  dt = (micros() - loop_timer) / 1000000.0;
  loop_timer = micros();

  // Sensör verilerini ve açıları hesapla (önceki kodla aynı)
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  float gyro_roll_input = (gyroY - gyro_y_cal) / 131.0;
  accel_angle_roll = atan(-accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * (180.0 / PI);
  angle_roll = 0.98 * (angle_roll + gyro_roll_input * dt) + 0.02 * accel_angle_roll;

  // PID Hesaplaması
  error_roll = angle_roll - 0; // Hedef: 0 derece
  integral_roll += error_roll * dt;
  derivative_roll = (error_roll - prev_error_roll) / dt;
  pid_output_roll = (kp_roll * error_roll) + (ki_roll * integral_roll) + (kd_roll * derivative_roll);
  prev_error_roll = error_roll;

  // Eğer motorlar aktifse, PID çıktısını motorlara gönder
  if (motors_active) {
    int motor_left_speed = tuning_throttle + pid_output_roll;
    int motor_right_speed = tuning_throttle - pid_output_roll;

    // Gerçek motor kontrol kodunu buraya yazarsın (servo.writeMicroseconds vb.)
    // Şimdilik sadece değerleri yazdırıyoruz
    // motor1.writeMicroseconds(motor_left_speed);
    // motor4.writeMicroseconds(motor_left_speed);
    // motor2.writeMicroseconds(motor_right_speed);
    // motor3.writeMicroseconds(motor_right_speed);
  } else {
    // Motorları durdur
    // motor1.writeMicroseconds(1000); // vb.
  }
  
  // Serial Plotter için veri yazdır
  Serial.print(0); // Hedef (mavi çizgi)
  Serial.print(" ");
  Serial.println(angle_roll); // Gerçek Açı (kırmızı çizgi)
}

void parseSerialCommand() {
  String command = Serial.readStringUntil('\n');
  char command_char = command.charAt(0);
  float value = command.substring(1).toFloat();

  switch(command_char) {
    case 'p':
      kp_roll = value;
      Serial.print("Yeni Kp degeri: ");
      Serial.println(kp_roll);
      break;
    case 'i':
      ki_roll = value;
      Serial.print("Yeni Ki degeri: ");
      Serial.println(ki_roll);
      break;
    case 'd':
      kd_roll = value;
      Serial.print("Yeni Kd degeri: ");
      Serial.println(kd_roll);
      break;
  }
  
  if (command.startsWith("start")) {
    motors_active = true;
    Serial.println("Motorlar test modunda AKTIF!");
  } else if (command.startsWith("stop")) {
    motors_active = false;
    integral_roll = 0; // Motorlar durunca integral birikimini sıfırla
    Serial.println("Motorlar DURDURULDU.");
  }
}