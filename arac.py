import RPi.GPIO as GPIO
import time

# GPIO pin numaralarını BCM moduna göre ayarla
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# --- Pin Tanımlamaları ---
# YÖN PINLERI
IN1_sol = 17; IN2_sol = 27 # Sol Ön Motor
IN3_sol = 22; IN4_sol = 23 # Sol Arka Motor
IN1_sag = 24; IN2_sag = 25 # Sağ Ön Motor
IN3_sag = 5;  IN4_sag = 6  # Sağ Arka Motor

# HIZ (PWM) PINLERI
ENA_sol = 13 # Sol Ön Motor Hız
ENB_sol = 12 # Sol Arka Motor Hız
ENA_sag = 19 # Sağ Ön Motor Hız
ENB_sag = 26 # Sağ Arka Motor Hız

# Pin listeleri
yon_pins = [IN1_sol, IN2_sol, IN3_sol, IN4_sol, IN1_sag, IN2_sag, IN3_sag, IN4_sag]
hiz_pins = [ENA_sol, ENB_sol, ENA_sag, ENB_sag]

# Pinleri çıkış olarak ayarla
for pin in yon_pins:
    GPIO.setup(pin, GPIO.OUT)
for pin in hiz_pins:
    GPIO.setup(pin, GPIO.OUT)

# --- PWM Nesneleri Oluşturma ---
# Frekans = 1000 Hz
p_sol_on = GPIO.PWM(ENA_sol, 1000)
p_sol_arka = GPIO.PWM(ENB_sol, 1000)
p_sag_on = GPIO.PWM(ENA_sag, 1000)
p_sag_arka = GPIO.PWM(ENB_sag, 1000)

# Tüm PWM'leri %0 hız ile başlat
p_sol_on.start(0)
p_sol_arka.start(0)
p_sag_on.start(0)
p_sag_arka.start(0)

# --- Hız Ayarları ---
hiz_tam = 80   # Tam hız (%100 çok ani olabilir, 80 ile başlamak iyi)
hiz_yavas = 40 # Yavaş hız (dönüşler için)

# --- Hareket Fonksiyonları ---

def motor_hizlari(sol_hiz, sag_hiz):
    p_sol_on.ChangeDutyCycle(sol_hiz)
    p_sol_arka.ChangeDutyCycle(sol_hiz)
    p_sag_on.ChangeDutyCycle(sag_hiz)
    p_sag_arka.ChangeDutyCycle(sag_hiz)

def dur():
    print("Duruyor...")
    motor_hizlari(0, 0)

def ileri():
    print("İleri gidiyor...")
    GPIO.output(IN1_sol, GPIO.HIGH); GPIO.output(IN2_sol, GPIO.LOW)
    GPIO.output(IN3_sol, GPIO.HIGH); GPIO.output(IN4_sol, GPIO.LOW)
    GPIO.output(IN1_sag, GPIO.HIGH); GPIO.output(IN2_sag, GPIO.LOW)
    GPIO.output(IN3_sag, GPIO.HIGH); GPIO.output(IN4_sag, GPIO.LOW)
    motor_hizlari(hiz_tam, hiz_tam)

def geri():
    print("Geri gidiyor...")
    GPIO.output(IN1_sol, GPIO.LOW); GPIO.output(IN2_sol, GPIO.HIGH)
    GPIO.output(IN3_sol, GPIO.LOW); GPIO.output(IN4_sol, GPIO.HIGH)
    GPIO.output(IN1_sag, GPIO.LOW); GPIO.output(IN2_sag, GPIO.HIGH)
    GPIO.output(IN3_sag, GPIO.LOW); GPIO.output(IN4_sag, GPIO.HIGH)
    motor_hizlari(hiz_tam, hiz_tam)

def ileri_sol():
    print("İleri sol kavis...")
    # Yönler: Hepsi ileri
    GPIO.output(IN1_sol, GPIO.HIGH); GPIO.output(IN2_sol, GPIO.LOW)
    GPIO.output(IN3_sol, GPIO.HIGH); GPIO.output(IN4_sol, GPIO.LOW)
    GPIO.output(IN1_sag, GPIO.HIGH); GPIO.output(IN2_sag, GPIO.LOW)
    GPIO.output(IN3_sag, GPIO.HIGH); GPIO.output(IN4_sag, GPIO.LOW)
    # Hızlar: Sol yavaş, Sağ hızlı
    motor_hizlari(hiz_yavas, hiz_tam)

def ileri_sag():
    print("İleri sağ kavis...")
    # Yönler: Hepsi ileri
    GPIO.output(IN1_sol, GPIO.HIGH); GPIO.output(IN2_sol, GPIO.LOW)
    GPIO.output(IN3_sol, GPIO.HIGH); GPIO.output(IN4_sol, GPIO.LOW)
    GPIO.output(IN1_sag, GPIO.HIGH); GPIO.output(IN2_sag, GPIO.LOW)
    GPIO.output(IN3_sag, GPIO.HIGH); GPIO.output(IN4_sag, GPIO.LOW)
    # Hızlar: Sol hızlı, Sağ yavaş
    motor_hizlari(hiz_tam, hiz_yavas)
    
def geri_sol():
    print("Geri sol kavis...")
    # Yönler: Hepsi geri
    GPIO.output(IN1_sol, GPIO.LOW); GPIO.output(IN2_sol, GPIO.HIGH)
    GPIO.output(IN3_sol, GPIO.LOW); GPIO.output(IN4_sol, GPIO.HIGH)
    GPIO.output(IN1_sag, GPIO.LOW); GPIO.output(IN2_sag, GPIO.HIGH)
    GPIO.output(IN3_sag, GPIO.LOW); GPIO.output(IN4_sag, GPIO.HIGH)
    # Hızlar: Sol yavaş, Sağ hızlı
    motor_hizlari(hiz_yavas, hiz_tam)

def geri_sag():
    print("Geri sağ kavis...")
    # Yönler: Hepsi geri
    GPIO.output(IN1_sol, GPIO.LOW); GPIO.output(IN2_sol, GPIO.HIGH)
    GPIO.output(IN3_sol, GPIO.LOW); GPIO.output(IN4_sol, GPIO.HIGH)
    GPIO.output(IN1_sag, GPIO.LOW); GPIO.output(IN2_sag, GPIO.HIGH)
    GPIO.output(IN3_sag, GPIO.LOW); GPIO.output(IN4_sag, GPIO.HIGH)
    # Hızlar: Sol hızlı, Sağ yavaş
    motor_hizlari(hiz_tam, hiz_yavas)

# --- Ana Program ---
try:
    print("Hareketler test ediliyor...")
    ileri()
    time.sleep(1)
    ileri_sol()
    time.sleep(2)
    ileri_sag()
    time.sleep(2)
    geri()
    time.sleep(1)
    geri_sol()
    time.sleep(2)
    geri_sag()
    time.sleep(2)
    dur()
    print("Test tamamlandı.")

except KeyboardInterrupt:
    print("Program sonlandırıldı.")
    dur()
    GPIO.cleanup()

finally:
    GPIO.cleanup()