"""
move.py - move the arm's claw tip to an (x, y, z) point. Runs on the desktop PC.

Put it in the same folder as IKtest.py (and close Thonny so the COM port is free).

    python move.py 12 0 3                 # one move, then exit
    python move.py 12 0 3 --hover         # go to z + 4 cm first, then down to z
    python move.py                        # interactive: type points one after another
    python move.py --dry-run 12 0 3       # print the angles only, nothing is sent
    python move.py --port COM4 --claw 100 12 0 3

Coordinates are in cm from the base rotation axis (the same frame as IKtest.py):
    x = forward, away from the robot     (must be >= 0, the base servo only covers 180 deg)
    y = see IKtest.py (BASE_SIGN decides which side is positive; check it on the arm)
    z = up, measured from the height of the base rotation axis (NOT from the table)
        Z_MIN_CM below is a TEMPORARY floor so the claw is not driven into the paper.

Interactive commands:  x y z   |   angles <claw> <elbow> <shoulder> <base>   |   claw <deg>
                       |   home   |   q
    "angles" sends raw servo angles with no IK, for measuring the real joint geometry.

Safety: if the point is unreachable the move is refused. If a joint would hit its
0/180 degree limit (so the claw would not really land on the point), you are shown
where the arm would end up and asked before anything is sent.
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import IKtest as ik
print("Using:", ik.__file__, "| L2 =", ik.L2)

try:
    import serial
except ImportError:
    serial = None

PORT = "COM10"
BAUD = 115200
DONE_TIMEOUT_S = 15.0
HOVER_CM = 4.0          # same hover height main.py uses above a block
FK_TOL_CM = 0.3         # round-trip error above this means a joint hit its limit
HOME = [0, 0, 0, 0]     # Claw, Elbow, Shoulder, Base (the Pico's boot pose)
# TEMPORARY floor: with the current IK constants a command of z = 8 left the claw tip about 1 cm
# above the paper, and z = 5 or lower drove it into the paper. Lower this once the IK is calibrated.
Z_MIN_CM = 8.0


def plan(x, y, z):
    """
    Return (angles_base_shoulder_elbow, warnings) for a target point.
    Raises ValueError if the point is out of reach.
    """
    base, shoulder, elbow = ik.solver(x, y, z)       # ValueError if unreachable
    warnings = []
    if z < Z_MIN_CM:
        warnings.append(
            f"z = {z:g} is below Z_MIN_CM = {Z_MIN_CM:g}: the claw tip will probably hit the paper "
            f"and jam. Raise z, or lower Z_MIN_CM at the top of move.py if you are sure."
        )
    if x < 0:
        warnings.append("x < 0 is behind the base; the base servo (0-180 deg) cannot point there.")
    try:
        fx, fy, fz = ik.fk(base, shoulder, elbow)
        err = math.dist((x, y, z), (fx, fy, fz))
        if err > FK_TOL_CM:
            warnings.append(
                f"A joint hits its 0/180 deg limit: the claw would end up near "
                f"({fx:.1f}, {fy:.1f}, {fz:.1f}), {err:.1f} cm from the target."
            )
    except (ValueError, ZeroDivisionError):
        warnings.append("Could not check the result with forward kinematics.")
    return (base, shoulder, elbow), warnings


class Pico:
    def __init__(self, port, baud, dry_run):
        self.dry_run = dry_run
        self.ser = None
        if not dry_run:
            if serial is None:
                sys.exit("pyserial is not installed (pip install pyserial), or use --dry-run.")
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
            except serial.SerialException as e:
                sys.exit(f"Could not open {port}: {e}\nIs the Pico plugged in and Thonny closed? "
                         f"(or use --dry-run)")
            time.sleep(1.0)
            self.ser.reset_input_buffer()
            print(f"Connected to {port} at {baud} baud.")

    def send(self, angles):
        """angles = [claw, elbow, shoulder, base]. Waits for DONE. Returns True on success."""
        line = ",".join(f"{a:.2f}" for a in angles)
        print(f"  angles  claw={angles[0]:.1f}  elbow={angles[1]:.1f}  "
              f"shoulder={angles[2]:.1f}  base={angles[3]:.1f}")
        if self.dry_run:
            print("  [dry run] nothing sent")
            return True
        self.ser.reset_input_buffer()
        t0 = time.perf_counter()
        self.ser.write((line + "\n").encode("utf-8"))
        self.ser.flush()
        while time.perf_counter() - t0 < DONE_TIMEOUT_S:
            raw = self.ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", "replace").strip()
            if text == "DONE":
                print(f"  done in {time.perf_counter() - t0:.2f} s (command reached, "
                      f"the servos give no position feedback)")
                return True
            if text:
                print(f"  [Pico] {text}")
        print(f"  No DONE within {DONE_TIMEOUT_S:.0f} s. Is the Pico running main.py?")
        return False

    def close(self):
        if self.ser is not None:
            self.ser.close()


def go_to(pico, x, y, z, claw, hover):
    """Plan and (after any needed confirmation) move to the point. Returns True if it moved."""
    points = [(x, y, z + HOVER_CM), (x, y, z)] if hover else [(x, y, z)]
    plans = []
    for px, py, pz in points:
        try:
            cmds, warns = plan(px, py, pz)
        except ValueError as e:
            print(f"  Refused ({px:g}, {py:g}, {pz:g}): {e}")
            return False
        plans.append(((px, py, pz), cmds, warns))

    all_warns = [(p, w) for p, _, ws in plans for w in ws]
    if all_warns:
        for p, w in all_warns:
            print(f"  WARNING at ({p[0]:g}, {p[1]:g}, {p[2]:g}): {w}")
        try:
            if input("  Move anyway? (y/n): ").strip().lower() not in ("y", "yes"):
                print("  Cancelled.")
                return False
        except EOFError:
            return False

    for (px, py, pz), (base, shoulder, elbow), _ in plans:
        print(f"Moving to ({px:g}, {py:g}, {pz:g}) cm")
        if not pico.send([claw, elbow, shoulder, base]):
            return False
    return True


def main():
    ap = argparse.ArgumentParser(description="Move the claw tip to an (x, y, z) point.")
    ap.add_argument("xyz", nargs="*", type=float, help="x y z in cm; leave empty for interactive mode")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--claw", type=float, default=0.0, help="claw angle in degrees (default 0)")
    ap.add_argument("--hover", action="store_true", help=f"approach from {HOVER_CM:g} cm above")
    ap.add_argument("--dry-run", action="store_true", help="print the angles, send nothing")
    args = ap.parse_args()

    if args.xyz and len(args.xyz) != 3:
        ap.error("give exactly three numbers: x y z")

    claw = max(0.0, min(180.0, args.claw))
    pico = Pico(args.port, args.baud, args.dry_run)
    try:
        if args.xyz:
            ok = go_to(pico, *args.xyz, claw, args.hover)
            sys.exit(0 if ok else 1)

        print("Type  x y z  (cm, from the base axis),  angles <claw> <elbow> <shoulder> <base>,")
        print("      claw <deg>,  home,  or q.")
        print(f"Link lengths: L0={ik.L0:g}, L1={ik.L1:g}, L2={ik.L2:g} cm. Claw angle: {claw:g} deg.")
        while True:
            try:
                text = input("move> ").strip().lower().replace(",", " ")
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not text:
                continue
            parts = text.split()
            if parts[0] in ("q", "quit", "exit"):
                break
            if parts[0] == "home":
                pico.send(HOME)
                continue
            if parts[0] == "angles":
                try:
                    vals = [float(p) for p in parts[1:]]
                    if len(vals) != 4:
                        raise ValueError
                except ValueError:
                    print("  angles needs four numbers: angles <claw> <elbow> <shoulder> <base>")
                    continue
                pico.send([max(0.0, min(180.0, v)) for v in vals])
                continue
            if parts[0] == "claw" and len(parts) == 2:
                try:
                    claw = max(0.0, min(180.0, float(parts[1])))
                except ValueError:
                    print("  claw needs a number, e.g.  claw 100")
                    continue
                print(f"  Claw angle set to {claw:g} deg (used on the next move).")
                continue
            try:
                x, y, z = (float(p) for p in parts)
                if len(parts) != 3:
                    raise ValueError
            except ValueError:
                print("  Enter three numbers (x y z), 'claw <deg>', 'home' or 'q'.")
                continue
            go_to(pico, x, y, z, claw, args.hover)
    except KeyboardInterrupt:
        print("\nStopped. The arm holds the last target it was given.")
    finally:
        pico.close()


if __name__ == "__main__":
    main()