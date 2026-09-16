from machine import Pin, PWM
import time 
# GPIO Pin setup for servo motors
Claw = PWM(Pin(8)) #claw servo motor is connected to Physical Pin 11
Mid_Claw = PWM(Pin(9)) #mid claw servo motor is connected to Physical Pin 12
Claw_base = PWM(Pin(10)) #claw base servo motor is connected to Physical Pin 14
Truebase = PWM(Pin(11)) #true base servo motor is connected to Physical Pin 15
# Claw Angles
Claw_angle = 0 #initial angle of the claw servo motor
Mid_Claw_angle = 0 #initial angle of the mid claw servo motor
Claw_base_angle = 0 #initial angle of the claw base servo motor
Truebase_angle = 0 #initial angle of the true base servo motor

# variable grouping for Bulk use 
servo_Motors = [Claw, Mid_Claw, Claw_base, Truebase] #grouping all the servo motors into a list for easy access
# setting up servo channels to 50Hz frequency
for s in servo_Motors:
    s.freq(50) #the frequency of the servo motor is 50Hz

# helper functions 
def set_pulse(servo_obj, pulse_us):
    """Sets pulse width in microseconds (440us to 2500us standard bounds)."""
    duty = int((pulse_us / 20000) * 65535)
    servo_obj.duty_u16(duty)

def set_angle(servo_obj, angle, min_us=440, max_us=2400):
    """Maps target angle (0° to 180°) directly to microsecond pulse width."""
    angle = max(0, min(180, angle))  # Clamp angle bounds
    pulse_us = min_us + (angle / 180) * (max_us - min_us)
    set_pulse(servo_obj, pulse_us)

# Calibration positions in microseconds (for yellow claw open and close useage)
OPEN = 920   
CLOSED = 440

#startup code 
print("setting all Servo motors to 0 degrees")
set_angle(Claw, 0)
set_angle(Mid_Claw, 0)
set_angle(Claw_base, 0)
set_angle(Truebase, 0)

while True:
    time.sleep(1)
    #Keeping motors in a loop to keep them active and prevent them from going into sleep mode
    set_angle(Claw, Claw_angle)
    set_angle(Mid_Claw, Mid_Claw_angle)
    set_angle(Claw_base, Claw_base_angle)
    set_angle(Truebase, Truebase_angle)