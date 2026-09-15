from machine import Pin, PWM
import time 

servo = PWM(Pin(8))
servo.freq(50) #the frequency of the servo motor is 50Hz

def set_pulse(pulse_us):
    duty = int((pulse_us / 20000) * 65535)
    servo.duty_u16(duty)

OPEN = 920   # your calibrated open position
CLOSED = 440 # your calibrated closed position

print("Moving to open (920us)...")
set_pulse(OPEN)
time.sleep(2)

print("Moving to closed (440us) and holding...")
set_pulse(CLOSED)

while True:
    time.sleep(1)