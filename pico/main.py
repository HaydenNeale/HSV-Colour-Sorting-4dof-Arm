from machine import Pin, PWM
import time 
# GPIO Pin setup for servo motors

Claw = PWM(Pin(8)) #claw servo motor is connected to Physical Pin 11
Mid_Claw = PWM(Pin(9)) #mid claw servo motor is connected to Physical Pin 12
Claw_base = PWM(Pin(10)) #claw base servo motor is connected to Physical Pin 14
Truebase = PWM(Pin(11)) #true base servo motor is connected to Physical Pin 15


# Claw Angles for servo motors and URART/I2c condition changing 
servo_Motors = [Claw, Mid_Claw, Claw_base, Truebase] # matches the order of target_angles and current_angles on arrays 

# smoothing code for servo motors
target_angles  = [0, 0, 0, 0]  # where you WANT each servo (index matches servo_Motors)
current_angles = [0, 0, 0, 0]  # where each servo actually IS right now
max_step = [4, 3, 2, 2]  # degrees per tick, tuned per joint -- heavier joints get smaller steps


# varible grouping for Bulk use 
servo_Motors = [Claw, Mid_Claw, Claw_base, Truebase] #grouping all the servo motors into a list for easy access
# setting up servo channels to 50Hz frequency
for s in servo_Motors:
    s.freq(50) #the frequency of the servo motor is 50Hz

# helper functions 
def set_pulse(servo_obj, pulse_us):
    """Sets pulse width in microseconds (440us to 2500us standard bounds)."""
    duty = int((pulse_us / 20000) * 65535)
    servo_obj.duty_u16(duty)

def set_angle(servo_obj, angle, min_us=500, max_us=2500):
    """Maps target angle (0° to 180°) directly to microsecond pulse width."""
    angle = max(0, min(180, angle))  # Clamp angle bounds
    pulse_us = min_us + (angle / 180) * (max_us - min_us)
    set_pulse(servo_obj, pulse_us)

# Claw open and closed pulse angles for gripping objects
OPEN = 920   
CLOSED = 440

#startup code 
print("setting all servo motors to 0 degrees")
for i in range(len(servo_Motors)):
    set_angle(servo_Motors[i], current_angles[i]) # loop for Sevo motors to set them to 0 degrees at startup

while True:
    time.sleep(0.02)  # Smoothness Tick speed 

    if current_angles == target_angles:
        pass  # already at target, do nothing holds angle if code hasnt changed the target angles
    else:
        for i in range(len(servo_Motors)):
            if current_angles[i] < target_angles[i]:
                current_angles[i] += min(max_step[i], target_angles[i] - current_angles[i])
            elif current_angles[i] > target_angles[i]:
                current_angles[i] -= min(max_step[i], current_angles[i] - target_angles[i])
            set_angle(servo_Motors[i], current_angles[i])
        # bob da builder?