from machine import Pin, PWM
import time 
# GPIO Pin setup for servo motors
Claw = PWM(Pin(8))        # claw servo motor is connected to Physical Pin 11
Mid_Claw = PWM(Pin(9))    # mid claw servo motor is connected to Physical Pin 12
Claw_base = PWM(Pin(10))  # claw base servo motor is connected to Physical Pin 14
Truebase = PWM(Pin(11))   # true base servo motor is connected to Physical Pin 15

# Reset Button setup (Connected between GP15 and GND)
reset_button = Pin(15, Pin.IN, Pin.PULL_UP)

# varible grouping for Bulk use 
# Claw Angles for servo motors and URART/I2c condition changing 
servo_Motors = [Claw, Mid_Claw, Claw_base, Truebase] # matches the order of target_angles and current_angles on arrays

# setting up servo channels to 50Hz frequency
for s in servo_Motors:
    s.freq(50) # the frequency of the servo motor is 50Hz


# smoothing code for servo motors
target_angles = [0, 0, 0, 0]   # where you WANT each servo (index matches servo_Motors)
current_angles = [0, 0, 0, 0]  # where each servo actually IS right now
max_step = [4, 3, 2, 2]        # degrees per tick, tuned per joint -- heavier joints get smaller steps

# Allowed operating bounds (min_deg, max_deg) per joint to prevent collisions and physical blocked zones
# [Claw, Mid_Claw, Claw_base, Truebase]
# BLOCKED_ZONES = [
#     (0, 180),    # Claw gripper range
#     (10, 170),   # Mid Claw (Elbow) range
#     (5, 175),    # Claw Base (Shoulder) range
#     (0, 180)     # Truebase range
# ]

# Pulse calibration bounds (microseconds)
MIN_US = 500    # 0 degrees
MID_US = 1500   # 90 degrees
MAX_US = 2500   # 180 degrees

# Claw open and closed pulse angles for gripping objects
OPEN = 920   
CLOSED = 440

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

def validate_and_set_target(servo_index, angle):
    """
    Validates target angle range and blocked zones. Sets angle directly if valid, 
    otherwise outputs an out-of-bounds warning and resets target to 0°.
    """
    # min_limit, max_limit = BLOCKED_ZONES[servo_index]
    min_limit, max_limit = 0, 180  # Default full servo range until blocked zones are measured
    if angle > max_limit or angle < min_limit:
        print(f"WARNING: Servo {servo_index} angle {angle}° out of bounds ({min_limit}°-{max_limit}°)! Resetting to 0°.")
        target_angles[servo_index] = 0
    else:
        target_angles[servo_index] = angle

def move_to_angles(new_targets):
    """
    Pass a list of target angles [Claw, Mid_Claw, Claw_base, Truebase].
    Updates targets safely while current position state is remembered.
    """
    processed_targets = list(new_targets)
    for i in range(len(processed_targets)):
        validate_and_set_target(i, processed_targets[i])


# Boot code to set all servo motors to 0 degrees at startup
print("setting all servo motors to 0 degrees")
for i in range(len(servo_Motors)):
    set_angle(servo_Motors[i], current_angles[i])

last_button_state = 1

# Main loop for smooth motion and reset button detection
while True:
    time.sleep(0.02)  # Smoothness Tick speed 

    # --- Reset Button Check ---
    # Detects button press (active LOW with internal pull-up)
    button_state = reset_button.value()
    if last_button_state == 1 and button_state == 0:
        print("\n[RESET BUTTON PRESSED] Returning all servos to 0°")
        move_to_angles([0, 0, 0, 0])
    last_button_state = button_state

    # --- Smooth Motion Interpolation ---
    if current_angles == target_angles:
        pass  # already at target, do nothing holds angle if code hasnt changed the target angles
    else:
        for i in range(len(servo_Motors)):
            if current_angles[i] < target_angles[i]:
                current_angles[i] += min(max_step[i], target_angles[i] - current_angles[i])
            elif current_angles[i] > target_angles[i]:
                current_angles[i] -= min(max_step[i], current_angles[i] - target_angles[i])
            
            set_angle(servo_Motors[i], current_angles[i])