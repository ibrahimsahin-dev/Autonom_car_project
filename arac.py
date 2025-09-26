import RPi.GPIO as GPIO
import time

# GPIO pin numaralarını BCM moduna göre ayarla
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# --- Pin Tanımlamaları ---
# SOL MOTORLAR (Sürücü 1)
IN1_sol = 17
IN2_sol = 27
IN3_sol = 22
IN4_sol = 23

# SAĞ MOTORLAR (Sürücü 2)
IN1_sag = 24
IN2_sag = 25
IN3_sag = 5
IN4_sag = 6

# Tüm motor pinlerini bir listede topla
motor_pins = [IN1_sol, IN2_sol, IN3_sol, IN4_sol, IN1_sag, IN2_sag, IN3_sag, IN4_sag]

# Pinleri çıkış olarak ayarla
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

# --- Hareket Fonksiyonları ---

def dur():
    print("Duruyor...")
    # Tüm pinlere LOW sinyali göndererek motorları durdur
    for pin in motor_pins:
        GPIO.output(pin, GPIO.LOW)

def ileri():
    print("İleri gidiyor...")
    # Sol motorlar ileri
    GPIO.output(IN1_sol, GPIO.HIGH)
    GPIO.output(IN2_sol, GPIO.LOW)
    GPIO.output(IN3_sol, GPIO.HIGH)
    GPIO.output(IN4_sol, GPIO.LOW)
    # Sağ motorlar ileri
    GPIO.output(IN1_sag, GPIO.HIGH)
    GPIO.output(IN2_sag, GPIO.LOW)
    GPIO.output(IN3_sag, GPIO.HIGH)
    GPIO.output(IN4_sag, GPIO.LOW)

def geri():
    print("Geri gidiyor...")
    # Sol motorlar geri
    GPIO.output(IN1_sol, GPIO.LOW)
    GPIO.output(IN2_sol, GPIO.HIGH)
    GPIO.output(IN3_sol, GPIO.LOW)
    GPIO.output(IN4_sol, GPIO.HIGH)
    # Sağ motorlar geri
    GPIO.output(IN1_sag, GPIO.LOW)
    GPIO.output(IN2_sag, GPIO.HIGH)
    GPIO.output(IN3_sag, GPIO.LOW)
    GPIO.output(IN4_sag, GPIO.HIGH)

def saga_don():
    print("Sağa dönüyor...")
    # Sol motorlar ileri (dönüşü sağlamak için)
    GPIO.output(IN1_sol, GPIO.HIGH)
    GPIO.output(IN2_sol, GPIO.LOW)
    GPIO.output(IN3_sol, GPIO.HIGH)
    GPIO.output(IN4_sol, GPIO.LOW)
    # Sağ motorlar geri
    GPIO.output(IN1_sag, GPIO.LOW)
    GPIO.output(IN2_sag, GPIO.HIGH)
    GPIO.output(IN3_sag, GPIO.LOW)
    GPIO.output(IN4_sag, GPIO.HIGH)

def sola_don():
    print("Sola dönüyor...")
    # Sol motorlar geri
    GPIO.output(IN1_sol, GPIO.LOW)
    GPIO.output(IN2_sol, GPIO.HIGH)
    GPIO.output(IN3_sol, GPIO.LOW)
    GPIO.output(IN4_sol, GPIO.HIGH)
    # Sağ motorlar ileri
    GPIO.output(IN1_sag, GPIO.HIGH)
    GPIO.output(IN2_sag, GPIO.LOW)
    GPIO.output(IN3_sag, GPIO.HIGH)
    GPIO.output(IN4_sag, GPIO.LOW)

# --- Ana Program ---
try:
    # Arabayı test etmek için bir dizi hareket
    ileri()
    time.sleep(2)  # 2 saniye ileri git

    geri()
    time.sleep(2)  # 2 saniye geri git
    
    sola_don()
    time.sleep(1) # 1 saniye sola dön
    
    saga_don()
    time.sleep(1) # 1 saniye sağa dön

    dur()

except KeyboardInterrupt:
    # Ctrl+C'ye basıldığında programı temiz bir şekilde sonlandır
    print("Program sonlandırıldı.")
    dur()
    GPIO.cleanup()

finally:
    # Her durumda program bittiğinde GPIO pinlerini temizle
    GPIO.cleanup()