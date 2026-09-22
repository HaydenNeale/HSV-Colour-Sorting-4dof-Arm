"""
4-DOF Robotic Arm - Inverse Kinematics Solver
================================================
Coordinate frame, measured from the BASE ROTATION AXIS:
    x = forward, away from the robot   (cm)
    y = left of the robot              (cm)   [positive y = target is to the arm's left]
    z = up, measured from the SAME HEIGHT as the base rotation axis (cm)
        (to place a real-world target: table height = base housing height + z)

Link lengths (your measurements, not the old reference code's numbers):
    L0 = 4 cm    base rotation axis -> shoulder joint (fixed, vertical, doesn't tilt)
    L1 = 8 cm    shoulder -> elbow
    L2 = 7 cm    elbow -> wrist
    CLAW = 5 cm  wrist -> gripper grasp point (assumed collinear with L2 -- verify
                 this, if the claw mounts at even a slight angle it needs its own offset)

CALIBRATION -- measured with a protractor at commanded angle = 90 on every joint
(the arm's physical "position zero"):
"""

import math

# ---------------------------------------------------------------------------
# MEASURED CONSTANTS -- update these as you confirm/correct them
# ---------------------------------------------------------------------------
L0 = 4.0
L1 = 8.0
L2 = 7.0 + 5.0  # wrist link + claw, assumed collinear -> effective reach

BASE_OFFSET_DEG = 3.0          # heading is 3 deg off "straight forward" at commanded 90
SHOULDER_FROM_VERTICAL_AT_ZERO = 30.0   # shoulder link's angle from vertical at commanded 90
ELBOW_INTERIOR_AT_ZERO = 160.0          # PLACEHOLDER -- confirm with a direct protractor
                                         # reading between L1 and L2 at commanded 90

# If a joint moves the WRONG direction relative to what solve_ik expects, flip
# its sign here instead of hunting through the math below.
BASE_SIGN = 1
SHOULDER_SIGN = 1
ELBOW_SIGN = 1

SERVO_MIN, SERVO_MAX = 0.0, 180.0


def clamp(angle):
    return max(SERVO_MIN, min(SERVO_MAX, angle))


def solve_ik(x, y, z):
    """
    Target point (x, y, z) in cm, relative to the base rotation axis.
    Returns (base_cmd, shoulder_cmd, elbow_cmd) as servo commands in degrees (0-180).
    Raises ValueError if the point is out of reach.
    """
    # Step 1: base rotation (azimuth)
    theta_base = math.degrees(math.atan2(y, x))
    base_cmd = clamp(90.0 + BASE_SIGN * (theta_base + BASE_OFFSET_DEG))

    # Step 2: collapse to the 2D shoulder/elbow plane
    r = math.hypot(x, y)          # horizontal distance from base axis
    h = z - L0                    # height above the shoulder joint
    D = math.hypot(r, h)          # straight-line distance, shoulder -> target

    reach_min, reach_max = abs(L1 - L2), L1 + L2
    if not (reach_min <= D <= reach_max):
        raise ValueError(
            f"Target unreachable: D={D:.2f}cm, reachable range is "
            f"{reach_min:.2f}cm to {reach_max:.2f}cm"
        )

    # Step 3: elbow interior angle, law of cosines (180 = fully straight)
    cos_elbow = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    theta_elbow = math.degrees(math.acos(cos_elbow))

    # Step 4: shoulder angle
    phi = math.degrees(math.atan2(h, r))
    cos_beta = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    beta = math.degrees(math.acos(cos_beta))
    shoulder_from_horizontal = phi + beta          # elbow-up solution
    shoulder_from_vertical = 90.0 - shoulder_from_horizontal

    # Step 5: apply calibration to get real servo commands
    shoulder_cmd = clamp(
        90.0 + SHOULDER_SIGN * (shoulder_from_vertical - SHOULDER_FROM_VERTICAL_AT_ZERO)
    )
    elbow_cmd = clamp(
        90.0 + ELBOW_SIGN * (theta_elbow - ELBOW_INTERIOR_AT_ZERO)
    )

    return base_cmd, shoulder_cmd, elbow_cmd


def forward_kinematics(base_cmd, shoulder_cmd, elbow_cmd):
    """
    Reverse of solve_ik -- given servo commands, compute where the end effector
    actually lands. Use this to sanity-check IK output against a ruler
    measurement on the real arm before trusting it.
    """
    theta_base = BASE_SIGN * (base_cmd - 90.0) - BASE_OFFSET_DEG

    shoulder_from_vertical = SHOULDER_SIGN * (shoulder_cmd - 90.0) + SHOULDER_FROM_VERTICAL_AT_ZERO
    shoulder_from_horizontal = 90.0 - shoulder_from_vertical

    theta_elbow = ELBOW_SIGN * (elbow_cmd - 90.0) + ELBOW_INTERIOR_AT_ZERO

    D = math.sqrt(L1**2 + L2**2 - 2 * L1 * L2 * math.cos(math.radians(theta_elbow)))
    cos_beta = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    beta = math.degrees(math.acos(cos_beta))
    phi = shoulder_from_horizontal - beta

    r = D * math.cos(math.radians(phi))
    h = D * math.sin(math.radians(phi))
    z = h + L0
    x = r * math.cos(math.radians(theta_base))
    y = r * math.sin(math.radians(theta_base))
    return x, y, z


if __name__ == "__main__":
    # Suggested first calibration target -- see chat for how to set this up physically.
    test_point = (12.0, 0.0, 5.0)  # cm: straight ahead, mid-reach, easy to measure to

    base_cmd, shoulder_cmd, elbow_cmd = solve_ik(*test_point)
    print(f"Target (x,y,z):        {test_point}")
    print(f"Servo commands:        base={base_cmd:.1f}  shoulder={shoulder_cmd:.1f}  elbow={elbow_cmd:.1f}")
    print(f"FK round-trip check:   {tuple(round(v, 2) for v in forward_kinematics(base_cmd, shoulder_cmd, elbow_cmd))}")
    print("^ this should equal the target above almost exactly -- if it doesn't, there's an algebra bug, not a calibration issue")