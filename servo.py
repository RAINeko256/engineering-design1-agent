import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

gp_out1 = 18
gp_out2 = 27
GPIO.setup(gp_out1, GPIO.OUT)
GPIO.setup(gp_out2, GPIO.OUT)

servo1 = GPIO.PWM(gp_out1, 50) 
servo2 = GPIO.PWM(gp_out2, 50)
#servo1
servo1.start(0.0)

servo1.ChangeDutyCycle(2.5)
time.sleep(1.0)

servo1.ChangeDutyCycle(12)
time.sleep(1.0)

servo1.stop()
time.sleep(1.0)
#servo2
servo2.start(0.0)

servo2.ChangeDutyCycle(2.5)
time.sleep(1.0)

servo2.ChangeDutyCycle(12)
time.sleep(1.0)

servo2.stop()


time.sleep(1.0)

GPIO.cleanup()