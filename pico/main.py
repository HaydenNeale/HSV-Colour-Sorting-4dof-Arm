from machine import Pin, PWM
import time 
import sys
import uselect 

# GPIO Pin setup for servo motors
Claw = PWM(Pin(8))        # claw servo motor is connected to Physical Pin 11
Elbow = PWM(Pin(9))    # elbow servo motor is connected to Physical Pin 12
Shoulder = PWM(Pin(10))  # shoulder servo motor is connected to Physical Pin 14
base = PWM(Pin(11))   # base servo motor is connected to Physical Pin 15

# varible grouping for Bulk use 
# Claw Angles for servo motors and URART/I2c condition changing 
servo_Motors = [Claw, Elbow, Shoulder, base] # matches the order of target_angles and current_angles on arrays

# setting up servo channels to 50Hz frequency
for s in servo_Motors:
    s.freq(50) # the frequency of the servo motor is 50Hz


# smoothing code for servo motors
target_angles = [0, 0, 0, 0]   # where you WANT each servo (index matches servo_Motors)
current_angles = [0, 0, 0, 0]  # where each servo actually IS right now
max_step = [4, 3, 2, 2]        # tick tuning servomotors max step

# Pulse calibration bounds (microseconds)
MIN_US = 500    # 0 degrees
MID_US = 1500   # 90 degrees
MAX_US = 2500   # 180 degrees

# HELPER FUNCTIONS
def set_pulse(servo_obj, pulse_us):
    """Sets pulse width in microseconds (440us to 2500us standard bounds)."""
    duty = int((pulse_us / 20000) * 65535)
    servo_obj.duty_u16(duty)

def set_angle(servo_obj, angle):
    """Maps target angle (0° to 180°) directly to microsecond pulse width."""
    clamped_angle = max(0, min(180, angle))  # Clamp angle bounds
    pulse_us = MIN_US + (clamped_angle / 180.0) * (MAX_US - MIN_US)
    set_pulse(servo_obj, pulse_us)

def move_to_angles(new_targets):
    """Updates target positions directly (validation pre-handled in main)."""
    for i in range(4):
        target_angles[i] = new_targets[i]

# serial setup for cross talk between control pc and pico
poll_obj = uselect.poll()
poll_obj.register(sys.stdin, uselect.POLLIN)

# Boot code to set all servo motors to 0 degrees at startup
print("setting all servo motors to 0 degrees")
for i in range(len(servo_Motors)):
    set_angle(servo_Motors[i], current_angles[i])

# Extra feature for tracking when the robot is moving
is_moving = False

# Main loop for smooth motion and reset button detection
# Main loop for smooth motion and reset button detection
while True:
    time.sleep(0.02)  # Smoothness Tick speed 

    # Extra feature for reading incoming serial commands from the PC
    if poll_obj.poll(0):
        line = sys.stdin.readline().strip()

        if line:
            try:
                # Extra feature for parsing incoming string formatted as "Claw,Elbow,Shoulder,Base"
                parts = [float(val) for val in line.split(',')]

                if len(parts) == 4:
                    move_to_angles(parts)
                    is_moving = True

            except ValueError:
                print("LOG: Invalid serial command format.")

    #Smooth Motion Interpolation
    motion_completed = True

    if current_angles == target_angles:
        pass  # already at target, do nothing holds angle if code hasnt changed the target angles
    else:
        for i in range(len(servo_Motors)):
            if current_angles[i] < target_angles[i]:
                current_angles[i] += min(max_step[i], target_angles[i] - current_angles[i])
                motion_completed = False

            elif current_angles[i] > target_angles[i]:
                current_angles[i] -= min(max_step[i], current_angles[i] - target_angles[i])
                motion_completed = False
            
            set_angle(servo_Motors[i], current_angles[i])

    # Extra feature for sending a completion signal back to the PC
    if is_moving and motion_completed:
        is_moving = False
        print("DONE")