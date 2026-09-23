"""
4-DOF Robotic Arm - Inverse Kinematics Solver
================================================
Coordinate frame, measured from the BASE ROTATION AXIS:
    x = forward, away from the robot   (cm)
    y = left of the robot              (cm)   [positive y = target is to the arm's left]
    z = up, measured from the SAME HEIGHT as the base rotation axis (cm)

Link lengths:
    L0 = 4 cm    base rotation axis -> shoulder joint
    L1 = 8 cm    shoulder -> elbow
    L2 = 10 cm   elbow -> claw center (7 cm link + 3 cm claw)
"""

import math

# ---------------------------------------------------------------------------
# MEASURED CONSTANTS & CALIBRATION OFFSETS
# ---------------------------------------------------------------------------
L0 = 4.0
L1 = 8.0
L2 = 7.0 + 5.0  # 7cm elbow-to-wrist + 5cm wrist-to-claw-End/Grippy part

# Claw vertical offset (cm above main arm axis)
CLAW_Z_OFFSET = 1

# Joint Angle Calibration
BASE_OFFSET_DEG = 0               # Base servo offset from straight ahead
SHOULDER_FROM_VERTICAL_AT_ZERO = 0 # Adjusted offset: shoulder is 10 deg from vertical at servo angle 90
ELBOW_INTERIOR_AT_ZERO = 40         # Interior angle between L1 and L2 at servo angle 90

# Joint Direction Multipliers (set to -1 if joint moves in reverse)
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
    """
    # Step 1: Base rotation angle (azimuth)
    theta_base = math.degrees(math.atan2(y, x))
    base_cmd = clamp(90.0 + BASE_SIGN * (theta_base + BASE_OFFSET_DEG))

    # Step 2: Plane projection and height offset adjustment
    r = math.hypot(x, y)                  # Horizontal distance from base center
    h = (z - CLAW_Z_OFFSET) - L0          # Height relative to shoulder joint
    D = math.hypot(r, h)                  # Direct distance from shoulder to target

    reach_min, reach_max = abs(L1 - L2), L1 + L2
    if not (reach_min <= D <= reach_max):
        raise ValueError(
            f"Target unreachable: D={D:.2f}cm. Reachable range: {reach_min:.2f}cm to {reach_max:.2f}cm"
        )

    # Step 3: Elbow interior angle (Law of Cosines)
    cos_elbow = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
    theta_elbow = math.degrees(math.acos(cos_elbow))

    # Step 4: Shoulder angle relative to horizon and vertical
    phi = math.degrees(math.atan2(h, r))
    cos_beta = (L1**2 + D**2 - L2**2) / (2 * L1 * D)
    beta = math.degrees(math.acos(cos_beta))
    shoulder_from_horizontal = phi + beta
    shoulder_from_vertical = 90.0 - shoulder_from_horizontal

    # Step 5: Convert to servo command angles using 10.0 degree zero offset
    shoulder_cmd = clamp(
        90.0 + SHOULDER_SIGN * (shoulder_from_vertical - SHOULDER_FROM_VERTICAL_AT_ZERO)
    )
    elbow_cmd = clamp(
        90.0 + ELBOW_SIGN * (theta_elbow - ELBOW_INTERIOR_AT_ZERO)
    )

    return base_cmd, shoulder_cmd, elbow_cmd


def forward_kinematics(base_cmd, shoulder_cmd, elbow_cmd):
    """
    Reverse kinematic calculation to verify resulting target (x, y, z).
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
    z = h + L0 + CLAW_Z_OFFSET
    x = r * math.cos(math.radians(theta_base))
    y = r * math.sin(math.radians(theta_base))
    return x, y, z


if __name__ == "__main__":
    # Test coordinates in cm
    test_point = (11.8, -0.7, 4)

    try:
        base_cmd, shoulder_cmd, elbow_cmd = solve_ik(*test_point)
        print(f"Target (x, y, z):      {test_point}")
        print(f"Servo commands:        base={base_cmd:.1f}  shoulder={shoulder_cmd:.1f}  elbow={elbow_cmd:.1f}")
        print(f"FK Round-trip Check:   {tuple(round(v, 2) for v in forward_kinematics(base_cmd, shoulder_cmd, elbow_cmd))}")
    except ValueError as e:
        print(e)