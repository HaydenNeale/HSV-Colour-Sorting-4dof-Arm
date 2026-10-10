import cv2
import time
import serial
import serial.tools.list_ports
import IKtest as ik
import hsvtest as hsv

# serial pico control
Port = 'COM13'  # com for the pico
baud = 115200

# Claw servo: 0 = closed. CLAW_OPEN is whatever value opens your claw.
CLAW_OPEN = 40
CLAW_CLOSED = 0

# Pose layout sent to the Pico: [claw, elbow, shoulder, base]
DROP = [CLAW_OPEN, 90, 0, 0]  # drop pose (claw value is overridden per step)
LIFT_CM = 3                   # height above the block for approach / lift
GRAB_Z_OFFSET_CM = 1          # aim this far above the detected z so the claw doesn't scrape the table
DESCENT_STEPS = 5             # extra waypoints between the approach point and the block


# Initialize serial connection
def connect_serial(port, baud_rate):
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to {port} at {baud_rate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error connecting to {port}: {e}")
        return None


def send_angles(ser, angles, timeout_s=15):
    """Send one pose and wait for the Pico's DONE. Returns True on success."""
    if ser is None:
        print(f"[PC SIMULATION] Moving to {angles}")
        time.sleep(1)
        return True
    if not ser.is_open:
        print("Serial connection is not open.")
        return False

    # Convert angles to a CSV string: "claw,elbow,shoulder,base\n"
    command_str = f"{angles[0]},{angles[1]},{angles[2]},{angles[3]}\n"
    ser.write(command_str.encode('utf-8'))
    print(f"[PC -> PICO] Sent angles: {angles}")

    # Wait for completion handshake from Pico (with a timeout so we never hang forever)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()
            if response == "DONE":
                print("[PICO -> PC] Target reached successfully.\n")
                return True
            elif response.startswith("LOG:"):
                print(f"[PICO LOG] {response}")
        time.sleep(0.01)

    print(f"Timed out after {timeout_s}s waiting for DONE.")
    return False


def with_claw(pose, claw):
    """Same joint angles, different claw value."""
    return [claw] + list(pose[1:])


def solve_pose(x, y, z, claw):
    """IK for one point. Returns [claw, elbow, shoulder, base], or None if unreachable."""
    try:
        base, shoulder, elbow = ik.solver(x, y, z)  # solver returns base, shoulder, elbow
    except ValueError as e:
        print(f"IK failed for ({x:.1f}, {y:.1f}, {z:.1f}): {e}")
        return None
    return [claw, elbow, shoulder, base]


def main():
    # Load camera calibration
    H, table_z_cm = hsv.load_calibration()

    # Load test image
    img = cv2.imread(hsv.IMAGE_PATH)
    if img is None:
        raise FileNotFoundError(f"Image not found: {hsv.IMAGE_PATH}")

    # Find blocks in the image: {'Red': (x, y, z), ...} (only detected colours)
    blocks = hsv.find_blocks(img, H, table_z_cm)

    # Solve IK for each detected colour; skip anything missing or unreachable
    jobs = []
    for color in ('Red', 'Green', 'Blue'):
        if color not in blocks:
            print(f"{color} block not detected - skipping.")
            continue

        x, y, z = blocks[color]
        z += GRAB_Z_OFFSET_CM

        # The grab point itself must be reachable, otherwise there is nothing to do.
        grab = solve_pose(x, y, z, CLAW_OPEN)
        if grab is None:
            print(f"Skipping {color} block: the grab point itself is out of reach.")
            continue

        # Vertical waypoints going UP from the block. Near the edge of the arm's reach,
        # going higher needs more elbow angle than the servo has (max 180), so stop at the
        # highest point that still solves and use a shorter approach/lift instead of
        # throwing the whole block away.
        n = DESCENT_STEPS + 1
        up = []  # poses above the block, lowest first
        for i in range(1, n + 1):
            pose = solve_pose(x, y, z + LIFT_CM * i / n, CLAW_OPEN)
            if pose is None:
                break
            up.append(pose)
        if not up:
            print(f"Skipping {color} block: can't get above it to approach/lift.")
            continue
        if len(up) < n:
            print(f"WARNING {color}: only {LIFT_CM * len(up) / n:.1f} of {LIFT_CM:.1f} cm clearance is "
                  f"reachable (block is near the edge of the arm's reach).")

        path = list(reversed(up)) + [grab]  # highest point first ... grab last

        # FK check. fk() takes (base, shoulder, elbow) = (pose[3], pose[2], pose[1])
        fx, fy, fz = ik.fk(grab[3], grab[2], grab[1])
        print(f"{color}: target=({x:.2f}, {y:.2f}, {z:.2f})  FK=({fx:.2f}, {fy:.2f}, {fz:.2f})")
        print(f"{color} grab IK: {grab}")

        jobs.append((color, path))

    ser = connect_serial(Port, baud)
    try:
        for color, path in jobs:
            above, grab = path[0], path[-1]
            mid = path[1:-1]
            # (label, pose, pause after move in seconds)
            steps = [("above block, claw open", above, 0.0)]
            steps += [(f"descend {i}, claw open", p, 0.0) for i, p in enumerate(mid, 1)]
            steps += [
                ("down to block, claw open", grab, 0.0),
                ("close claw", with_claw(grab, CLAW_CLOSED), 0.5),  # let the grip settle
            ]
            steps += [(f"rise {i}, claw closed", with_claw(p, CLAW_CLOSED), 0.0)
                      for i, p in enumerate(reversed(mid), 1)]
            steps += [
                ("lift, claw closed", with_claw(above, CLAW_CLOSED), 0.0),
                ("to drop, claw closed", with_claw(DROP, CLAW_CLOSED), 0.0),
                ("release", with_claw(DROP, CLAW_OPEN), 0.5),
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