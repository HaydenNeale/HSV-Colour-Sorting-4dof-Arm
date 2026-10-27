"""
4-DOF Robotic Arm - Inverse and Forward Kinematics

Coordinate frame:

    x = forward from robot (cm)
    y = right from robot (cm)
    z = height above tabletop (cm)

Servo reference pose:

    base     = 90° -> faces forward
    shoulder = 90° -> assumed horizontal reference
    elbow    = 90° -> assumed calibration reference

L0 = shoulder pivot height above tabletop.
"""

import math

# ============================================================
# MEASURED GEOMETRY (cm)
# ============================================================

# Servo calibration offsets
SHOULDER_OFFSET_DEG = 0.0
ELBOW_OFFSET_DEG = 0.0

L0 = 13.0

L1 = 15.0

FOREARM = 8.0
CLAW = 4.0
L2 = FOREARM + CLAW       # 12 cm with the values above


# ============================================================
# SERVO CALIBRATION
# ============================================================

# Base:
# 0 = right, 90 = forward, 180 = left
BASE_OFFSET_DEG = 0.0

# Shoulder:
# 90 = L1 horizontal
# Increasing command raises L1 when SHOULDER_SIGN = +1
SHOULDER_SIGN = -1

# Elbow:
# 90 = L2 vertical downward relative to horizontal L1
# 180 = links straightened in the outward direction
# 0 = opposite folded configuration
ELBOW_SIGN = 1

BASE_SIGN = 1

SERVO_MIN = 0.0
SERVO_MAX = 180.0
_TOL = 0.01


# ============================================================
# HELPERS
# ============================================================

def _acos_deg(c):
    """acos in degrees, safe against floating-point overshoot."""
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _check_servo(angle, name):
    """Reject servo commands outside the permitted range."""
    if not (SERVO_MIN - _TOL <= angle <= SERVO_MAX + _TOL):
        raise ValueError(
            f"{name} requires {angle:.2f} degrees; "
            f"allowed range is {SERVO_MIN:.0f}-{SERVO_MAX:.0f}"
        )

    return max(SERVO_MIN, min(SERVO_MAX, angle))


# ============================================================
# INVERSE KINEMATICS
# ============================================================

def solver(x, y, z):
    """
    Convert a target position (x, y, z) in cm into servo commands.

    Returns:
        (base_cmd, shoulder_cmd, elbow_cmd)

    The elbow branch is chosen to match the reference pose:
        shoulder = 90, elbow = 90
        L1 horizontal, L2 pointing down.
    """

    # --------------------------------------------------------
    # 1. Base rotation
    # --------------------------------------------------------

    # Positive y means right, so the base command decreases
    # as the target moves to the right.
    theta_base = math.degrees(math.atan2(y, x))

    base_cmd = _check_servo(
        90.0 + BASE_SIGN * (theta_base + BASE_OFFSET_DEG),
        "base"
    )

    # --------------------------------------------------------
    # 2. Convert target to the arm's 2D working plane
    # --------------------------------------------------------

    r = math.hypot(x, y)

    # Shoulder-pivot-relative target height.
    h = z - L0 
    D = math.hypot(r, h)

    # --------------------------------------------------------
    # 3. Reachability check
    # --------------------------------------------------------

    reach_min = abs(L1 - L2)
    reach_max = L1 + L2

    if D < reach_min - 1e-6 or D > reach_max + 1e-6:
        raise ValueError(
            f"Target unreachable: distance={D:.2f} cm. "
            f"Reachable distance range is "
            f"{reach_min:.2f}-{reach_max:.2f} cm."
        )

    if D < 1e-9:
        raise ValueError("Target is at the shoulder pivot; IK is undefined.")

    # --------------------------------------------------------
    # 4. Calculate the relative angle between the two links
    # --------------------------------------------------------

    # Law of cosines:
    # q_rel = 0 degrees means both links point in the same direction.
    cos_q = (
        D**2 - L1**2 - L2**2
    ) / (2.0 * L1 * L2)

    q_magnitude = _acos_deg(cos_q)

    # Negative branch matches the reference configuration where
    # L1 is horizontal and L2 points downward at elbow command 90.
    q_rel = -q_magnitude

    # --------------------------------------------------------
    # 5. Calculate the shoulder's geometric angle
    # --------------------------------------------------------

    target_angle = math.degrees(math.atan2(h, r))

    correction = math.degrees(math.atan2(
        L2 * math.sin(math.radians(q_rel)),
        L1 + L2 * math.cos(math.radians(q_rel))
    ))

    # Angle of L1 relative to the horizontal.
    shoulder_angle = target_angle - correction

    # --------------------------------------------------------
    # 6. Convert geometric angles to servo commands
    # --------------------------------------------------------

    shoulder_cmd = _check_servo(
        90.0 + SHOULDER_SIGN * shoulder_angle,
        "shoulder"
    )

    # Reference calibration:
    # q_rel = -90 degrees -> elbow servo = 90 degrees.
    #
    # q_rel = 0 degrees -> elbow servo = 180 degrees
    # when ELBOW_SIGN = +1.
    elbow_cmd = _check_servo(
        90.0 + ELBOW_SIGN * (q_rel + 90.0),
        "elbow"
    )

    return base_cmd, shoulder_cmd, elbow_cmd


# ============================================================
# FORWARD KINEMATICS
# ============================================================

def fk(base_cmd, shoulder_cmd, elbow_cmd):
    """
    Calculate the predicted (x, y, z) position from servo commands.

    Argument order:
        base, shoulder, elbow

    This uses the same geometry and calibration as solver().
    """

    # --------------------------------------------------------
    # 1. Recover the base angle
    # --------------------------------------------------------

    theta_base = (
        BASE_SIGN * (base_cmd - 90.0)
        - BASE_OFFSET_DEG
    )

    # --------------------------------------------------------
    # 2. Recover link angles from the servo commands
    # --------------------------------------------------------

    shoulder_angle = (
        SHOULDER_SIGN * (shoulder_cmd - 90.0)
    )

    q_rel = (
        ELBOW_SIGN * (elbow_cmd - 90.0)
        - 90.0
    )

    # Absolute angle of L2 in the vertical working plane.
    forearm_angle = shoulder_angle + q_rel

    # --------------------------------------------------------
    # 3. Calculate the endpoint relative to the shoulder pivot
    # --------------------------------------------------------

    r = (
        L1 * math.cos(math.radians(shoulder_angle))
        + L2 * math.cos(math.radians(forearm_angle))
    )

    h = (
        L1 * math.sin(math.radians(shoulder_angle))
        + L2 * math.sin(math.radians(forearm_angle))
    )

    # Convert back to the base-axis coordinate frame.
    z = h + L0

    x = r * math.cos(math.radians(theta_base))
    y = r * math.sin(math.radians(theta_base))

    return x, y, z


# ============================================================
# TEST
# ============================================================

if __name__ == "__main__":

    test_point = (10.0, 10.0, 10.0)

    try:
        base_cmd, shoulder_cmd, elbow_cmd = solver(*test_point)

        print(f"Target: {test_point}")
        print(
            f"Servo commands: "
            f"base={base_cmd:.1f}, "
            f"shoulder={shoulder_cmd:.1f}, "
            f"elbow={elbow_cmd:.1f}"
        )

        predicted = fk(base_cmd, shoulder_cmd, elbow_cmd)

        error = math.dist(test_point, predicted)

        print(
            "FK position: "
            f"({predicted[0]:.2f}, "
            f"{predicted[1]:.2f}, "
            f"{predicted[2]:.2f})"
        )

        print(f"Round-trip error: {error:.4f} cm")

    except ValueError as e:
        print(f"IK error: {e}")

