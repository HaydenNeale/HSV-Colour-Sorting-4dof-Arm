"""
4-DOF Robotic Arm - Inverse Kinematics solver
================================================
Coordinate frame, measured from the BASE ROTATION AXIS:
    x = forward, away from the robot   (cm)
    y = left of the robot              (cm)   [positive y = target is to the arm's left]
    z = up, measured from the SAME HEIGHT as the base rotation axis (cm)

Link lengths:
    L0 = 4 cm    base rotation axis -> shoulder joint
    L1 = 8 cm    shoulder -> elbow
    L2 = 13 cm   elbow -> claw tip (7 cm link + 5 cm to claw middle + 1 cm to tip)

solver() raises ValueError if the target is out of reach OR if it would need a
servo angle outside 0-180 (it no longer clamps silently, because a clamped angle
means the arm goes somewhere other than where you asked).
"""

import math

# ---------------------------------------------------------------------------
# MEASURED CONSTANTS & CALIBRATION OFFSETS
# ---------------------------------------------------------------------------
L0 = 4.0
L1 = 8.0
# Extra distance from the middle of the claw to the claw tip, along the forearm
# direction. Set to 0 to target the middle of the claw again.
CLAW_TIP_EXTRA = 1.0
# VERIFY WITH A TAPE MEASURE: elbow pivot -> middle of the claw is 7 + 5 = 12 cm,
# plus CLAW_TIP_EXTRA to land the claw tip on the target.
L2 = 7.0 + 5.0 + CLAW_TIP_EXTRA

# Claw vertical offset (cm above main arm axis)
CLAW_Z_OFFSET = 2

# Joint angle calibration. "Neutral" = servo command of 90 degrees.
BASE_OFFSET_DEG = 0                    # base servo offset from straight ahead
SHOULDER_FROM_VERTICAL_AT_NEUTRAL = 0  # shoulder angle from vertical when servo = 90
ELBOW_INTERIOR_AT_NEUTRAL = 40         # interior angle between L1 and L2 when servo = 90

# Joint direction multipliers (set to -1 if a joint moves in reverse)
BASE_SIGN = 1
SHOULDER_SIGN = 1
ELBOW_SIGN = 1

SERVO_MIN, SERVO_MAX = 0.0, 180.0
_TOL = 0.01  # degrees of float slack allowed at the servo limits


def _acos_deg(c):
    """acos in degrees, safe against tiny float overshoot at the edge of reach."""
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _check_servo(angle, name):
    if not (SERVO_MIN - _TOL <= angle <= SERVO_MAX + _TOL):
        raise ValueError(
            f"{name} would need servo angle {angle:.1f} deg, outside {SERVO_MIN:.0f}-{SERVO_MAX:.0f}"
        )
    return max(SERVO_MIN, min(SERVO_MAX, angle))


def solver(x, y, z):
    """
    Target point (x, y, z) in cm, relative to the base rotation axis.
    Returns (base_cmd, shoulder_cmd, elbow_cmd) as servo commands in degrees (0-180).
    """
    # Step 1: Base rotation angle (azimuth)
    theta_base = math.degrees(math.atan2(y, x))
    base_cmd = _check_servo(90.0 + BASE_SIGN * (theta_base + BASE_OFFSET_DEG), "base")

    # Step 2: Plane projection and height offset adjustment
    r = math.hypot(x, y)                  # Horizontal distance from base centre
    h = (z - CLAW_Z_OFFSET) - L0          # Height relative to shoulder joint
    D = math.hypot(r, h)                  # Direct distance from shoulder to target

    reach_min, reach_max = abs(L1 - L2), L1 + L2
    if not (reach_min <= D <= reach_max):
        raise ValueError(
            f"Target unreachable: D={D:.2f}cm. Reachable range: {reach_min:.2f}cm to {reach_max:.2f}cm"
        )

    # Step 3: Elbow interior angle (law of cosines)
    theta_elbow = _acos_deg((L1**2 + L2**2 - D**2) / (2 * L1 * L2))

    # Step 4: Shoulder angle relative to horizon and vertical (elbow-up solution)
    phi = math.degrees(math.atan2(h, r))
    beta = _acos_deg((L1**2 + D**2 - L2**2) / (2 * L1 * D))
    shoulder_from_horizontal = phi + beta
    shoulder_from_vertical = 90.0 - shoulder_from_horizontal

    # Step 5: Convert to servo command angles
    shoulder_cmd = _check_servo(
        90.0 + SHOULDER_SIGN * (shoulder_from_vertical - SHOULDER_FROM_VERTICAL_AT_NEUTRAL),
        "shoulder",
    )
    elbow_cmd = _check_servo(
        90.0 + ELBOW_SIGN * (theta_elbow - ELBOW_INTERIOR_AT_NEUTRAL),
        "elbow",
    )

    return base_cmd, shoulder_cmd, elbow_cmd


def fk(base_cmd, shoulder_cmd, elbow_cmd):
    """
    Forward kinematics to verify the resulting target (x, y, z).
    NOTE the argument order: base, shoulder, elbow (same as solver()'s return order).
    """
    theta_base = BASE_SIGN * (base_cmd - 90.0) - BASE_OFFSET_DEG

    shoulder_from_vertical = SHOULDER_SIGN * (shoulder_cmd - 90.0) + SHOULDER_FROM_VERTICAL_AT_NEUTRAL
    shoulder_from_horizontal = 90.0 - shoulder_from_vertical

    theta_elbow = ELBOW_SIGN * (elbow_cmd - 90.0) + ELBOW_INTERIOR_AT_NEUTRAL

    D = math.sqrt(L1**2 + L2**2 - 2 * L1 * L2 * math.cos(math.radians(theta_elbow)))
    beta = _acos_deg((L1**2 + D**2 - L2**2) / (2 * L1 * D))
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
        base_cmd, shoulder_cmd, elbow_cmd = solver(*test_point)
        print(f"Target (x, y, z):      {test_point}")
        print(f"Servo commands:        base={base_cmd:.1f}  shoulder={shoulder_cmd:.1f}  elbow={elbow_cmd:.1f}")
        print(f"FK round-trip check:   {tuple(round(v, 2) for v in fk(base_cmd, shoulder_cmd, elbow_cmd))}")
    except ValueError as e:
        print(e)