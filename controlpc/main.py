import cv2
import time
import serial
import serial.tools.list_ports
import IKtest as ik
import hsvtest as hsv

# ============================================================
# SERIAL CONTROL
# ============================================================

Port = 'COM10'
baud = 115200

# ============================================================
# CLAW
# ============================================================

CLAW_OPEN = 40
CLAW_CLOSED = 0

# ============================================================
# PICKING HEIGHTS (cm, measured above the tabletop)
# ============================================================

GRAB_Z_CM = 7.0       # ALWAYS command the block-grab point at z = 9 cm
LIFT_CM = 3.0         # Approach height above the grab point
DESCENT_STEPS = 5     # Number of intermediate descent/lift waypoints

APPROACH_Z_CM = GRAB_Z_CM + LIFT_CM  # 12 cm above tabletop

# Manual servo pose for the drop location.
# This is a joint-space pose, NOT an IK-generated Cartesian target.
DROP = [CLAW_OPEN, 90, 0, 0]


# ============================================================
# SERIAL HELPERS
# ============================================================

def connect_serial(port, baud_rate):
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to {port} at {baud_rate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error connecting to {port}: {e}")
        return None


def send_angles(ser, angles, timeout_s=15):
    """Send [claw, elbow, shoulder, base] and wait for DONE."""

    if ser is None:
        print(f"[PC SIMULATION] Moving to {angles}")
        time.sleep(1)
        return True

    if not ser.is_open:
        print("Serial connection is not open.")
        return False

    command_str = f"{angles[0]},{angles[1]},{angles[2]},{angles[3]}\n"
    ser.write(command_str.encode("utf-8"))
    print(f"[PC -> PICO] Sent angles: {angles}")

    deadline = time.time() + timeout_s

    while time.time() < deadline:
        if ser.in_waiting > 0:
            response = ser.readline().decode("utf-8").strip()

            if response == "DONE":
                print("[PICO -> PC] Target reached successfully.\n")
                return True

            elif response.startswith("LOG:"):
                print(f"[PICO LOG] {response}")

        time.sleep(0.01)

    print(f"Timed out after {timeout_s}s waiting for DONE.")
    return False


def with_claw(pose, claw):
    """Replace the claw command without changing the joint angles."""
    return [claw] + list(pose[1:])


# ============================================================
# INVERSE KINEMATICS
# ============================================================

def solve_pose(x, y, z, claw):
    """
    IKtest.solver() returns (base, shoulder, elbow).

    The Pico expects:
        [claw, elbow, shoulder, base]

    x, y, z are in cm, with z measured above the tabletop.
    """

    try:
        base, shoulder, elbow = ik.solver(x, y, z)

    except ValueError as e:
        print(
            f"IK failed for "
            f"({x:.2f}, {y:.2f}, {z:.2f}): {e}"
        )
        return None

    return [
        claw,
        round(elbow),
        round(shoulder),
        round(base)
    ]


# ============================================================
# MAIN PICK-AND-PLACE SEQUENCE
# ============================================================

def main():

    # Load camera calibration.
    H, table_z_cm = hsv.load_calibration()

    # Load image.
    img = cv2.imread(hsv.IMAGE_PATH)

    if img is None:
        raise FileNotFoundError(
            f"Image not found: {hsv.IMAGE_PATH}"
        )

    # Detect blocks.
    # Expected format: {'Red': (x, y, z), ...}
    blocks = hsv.find_blocks(img, H, table_z_cm)

    jobs = []

    # Process the detected colours.
    for color in ("Red", "Green", "Blue"):

        if color not in blocks:
            print(f"{color} block not detected - skipping.")
            continue

        # Use detected x and y, but FORCE the target height to 9 cm.
        detected_x, detected_y, detected_z = blocks[color]

        x = float(detected_x)
        y = float(detected_y)
        z = GRAB_Z_CM

        print(f"\n--- {color} block ---")
        print(
            f"Camera estimate: x={detected_x:.2f}, "
            f"y={detected_y:.2f}, z={detected_z:.2f} cm"
        )
        print(
            f"IK grab target:  x={x:.2f}, "
            f"y={y:.2f}, z={z:.2f} cm"
        )

        # ----------------------------------------------------
        # 1. Solve the grab pose at exactly z = 9 cm.
        # ----------------------------------------------------

        grab = solve_pose(x, y, GRAB_Z_CM, CLAW_OPEN)

        if grab is None:
            print(
                f"Skipping {color}: grab point at "
                f"z={GRAB_Z_CM:.1f} cm is unreachable."
            )
            continue

        # ----------------------------------------------------
        # 2. Build approach waypoints above the grab point.
        #    All waypoints use the same IK coordinate system.
        # ----------------------------------------------------

        up = []

        for i in range(1, DESCENT_STEPS + 1):

            waypoint_z = (
                GRAB_Z_CM
                + LIFT_CM * i / DESCENT_STEPS
            )

            pose = solve_pose(x, y, waypoint_z, CLAW_OPEN)

            if pose is None:
                print(
                    f"WARNING {color}: cannot reach "
                    f"approach waypoint z={waypoint_z:.2f} cm."
                )
                break

            # Stored from lowest to highest.
            up.append(pose)

        if not up:
            print(
                f"Skipping {color}: no reachable approach "
                f"waypoints above the grab point."
            )
            continue

        if len(up) < DESCENT_STEPS:
            highest_z = (
                GRAB_Z_CM
                + LIFT_CM * len(up) / DESCENT_STEPS
            )
            print(
                f"WARNING {color}: highest planned waypoint "
                f"is approximately z={highest_z:.2f} cm, "
                f"not the full {APPROACH_Z_CM:.1f} cm."
            )

        # Highest point first, then descend to the grab pose.
        path = list(reversed(up)) + [grab]

        # ----------------------------------------------------
        # 3. Forward-kinematics diagnostic.
        #    This checks mathematical consistency, not physical
        #    servo calibration.
        # ----------------------------------------------------

        fx, fy, fz = ik.fk(
            grab[3],  # base
            grab[2],  # shoulder
            grab[1]   # elbow
        )

        print(f"Grab servo pose [claw, elbow, shoulder, base]: {grab}")
        print(
            f"FK check: x={fx:.2f}, y={fy:.2f}, z={fz:.2f} cm"
        )
        print(
            f"FK height error relative to z=9: "
            f"{fz - GRAB_Z_CM:+.2f} cm"
        )

        jobs.append((color, path))

    # --------------------------------------------------------
    # 4. Execute pick-and-place jobs.
    # --------------------------------------------------------

    if not jobs:
        print("No reachable blocks to pick.")
        return

    ser = connect_serial(Port, baud)

    try:
        for color, path in jobs:

            above = path[0]
            grab = path[-1]
            mid = path[1:-1]

            steps = [
                ("above block, claw open", above, 0.0)
            ]

            steps += [
                (f"descend {i}, claw open", pose, 0.0)
                for i, pose in enumerate(mid, 1)
            ]

            steps += [
                ("down to z=9 cm grab target", grab, 0.0),
                (
                    "close claw",
                    with_claw(grab, CLAW_CLOSED),
                    0.5
                )
            ]

            steps += [
                (
                    f"rise {i}, claw closed",
                    with_claw(pose, CLAW_CLOSED),
                    0.0
                )
                for i, pose in enumerate(reversed(mid), 1)
            ]

            steps += [
                (
                    "lift, claw closed",
                    with_claw(above, CLAW_CLOSED),
                    0.0
                ),
                (
                    "to drop, claw closed",
                    with_claw(DROP, CLAW_CLOSED),
                    0.0
                ),
                (
                    "release",
                    with_claw(DROP, CLAW_OPEN),
                    0.5
                )
            ]

            for label, pose, pause in steps:

                print(f"{color} - {label}: {pose}")

                if not send_angles(ser, pose):
                    print("Aborting sequence.")
                    return

                if pause:
                    time.sleep(pause)

    finally:
        if ser is not None:
            ser.close()


if __name__ == "__main__":
    main()