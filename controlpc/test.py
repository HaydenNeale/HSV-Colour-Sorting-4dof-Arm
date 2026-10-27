"""
main_test.py - PC-side test runner for the 4-DOF arm.
=====================================================
Run this on the desktop PC (NOT on the Pico), in the same folder as main.py,
IKtest.py, hsvtest.py and calibration.json:

    python main_test.py                  # real arm on PORT below
    python main_test.py --port COM10     # choose the port
    python main_test.py --sim            # dry run with a modelled Pico (no arm)

Nothing starts by itself. You pick a test from the menu, read what to prepare,
and press Enter to start it. When a test finishes you are returned to the menu.
Results are saved as CSV files in ./test_results/ (files from a dry run have
"_SIM" in the name and must NOT be used as report data).

Tests
  1  Serial protocol check (DONE handshake, invalid command, wrong-length command)
  2  Smoothing speed / timing per joint (checks max_step, 20 ms tick, Table 1)
  3  Commanded vs measured angle (+ duty cycle) up/down sweep (linearity, hysteresis)
  4  Held-pose multimeter readings (current / voltage, ideal vs active load)
  5  IK / FK round trip and base-angle mapping (no arm needed)
  6  Vision: detected vs measured block position
  7  All four joints at once: current vs distance, out and back to zero

SAFETY: the LIMITS and poses below are guesses. Set them to what your arm can
really do without hitting the table, the base or itself BEFORE running tests
that move the arm. Ctrl+C stops a test; the arm then holds its last pose.
"""

import argparse
import csv
import math
import os
import statistics
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    serial = None

try:
    import IKtest as ik
except ImportError:
    ik = None

try:
    import cv2
    import hsvtest as hsv
except ImportError:
    cv2 = None
    hsv = None

# ---------------------------------------------------------------------------
# SETTINGS (edit these)
# ---------------------------------------------------------------------------
PORT = "COM10"
BAUD = 115200

JOINTS = ["Claw", "Elbow", "Shoulder", "Base"]      # order sent to the Pico
PICO_PINS = ["GP8", "GP9", "GP10", "GP11"]
MAX_STEP = [4, 3, 2, 2]     # must match max_step in the Pico main.py
TICK_S = 0.02               # must match time.sleep(0.02) in the Pico main.py

HOME = [0, 0, 0, 0]         # the Pico's boot pose
DROP = [100, 90, 0, 0]      # drop pose used by main.py

# SAFE test range per joint (min, max) in degrees. CHECK THESE ON YOUR ARM.
LIMITS = [(0, 100), (0, 90), (0, 90), (0, 180)]

REST_POSE = HOME            # where the other joints sit while one joint is tested
SPANS = (20, 60)            # test 2: two move sizes (deg) per joint
REPEATS = 3                 # test 2: repeats per direction and span
ANGLE_POINTS = [0, 45, 90, 135, 180]    # test 3: commanded angles
SETTLE_S = 1.0              # wait after DONE before reading the arm (test 3)
HOLD_SETTLE_S = 3.0         # wait for current to settle (test 4)
READINGS = 5                # test 4 and 7: meter readings per state
# Test 7: every joint moves at once to this fraction of its LIMITS range (0 = HOME), then back.
CURRENT_FRACTIONS = [0.25, 0.5, 0.75, 1.0]
MOVE_TIMEOUT_S = 15.0

# Test 4 states: (name, pose or None, note). None = no movement (manual set-up).
CURRENT_STATES = [
    ("Pico only (servos disconnected)", None,
     "Unplug the servo power so only the Pico is on the measured line."),
    ("Servos holding HOME", HOME, ""),
    ("Servos holding DROP pose", DROP, ""),
    # Add poses here, e.g. a shoulder series for the gravity-load curve:
    # ("Shoulder 30 deg, block held", [0, 90, 30, 90], "Put a block in the claw first."),
]

# Test 5 grid (cm, from the base rotation axis)
IK_X = [-5, 4, 8, 12, 16]   # -5 is behind the base on purpose (shows the clamp)
IK_Y = [-15, -10, -5, 0, 5, 10, 15]


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
class StopTest(Exception):
    """Raised to leave the current test early (user typed q, or input ended)."""


def ask(prompt):
    try:
        return input(prompt)
    except EOFError:
        raise StopTest()


def confirm(prompt):
    while True:
        s = ask(prompt + " (y/n): ").strip().lower()
        if s in ("y", "yes"):
            return True
        if s in ("n", "no"):
            return False


def ask_float(prompt):
    """Number, or None if Enter is pressed. 'q' leaves the test."""
    while True:
        s = ask(prompt).strip()
        if s == "":
            return None
        if s.lower() in ("q", "quit"):
            raise StopTest()
        try:
            return float(s)
        except ValueError:
            print("  Enter a number, press Enter to skip, or q to stop this test.")


def mean_sd(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None, None, 0
    sd = statistics.stdev(vals) if len(vals) > 1 else 0.0
    return statistics.mean(vals), sd, len(vals)


def f(x, nd=2):
    return "-" if x is None else f"{x:.{nd}f}"


def clamp_angle(a):
    return max(0.0, min(180.0, float(a)))


def pulse_us(angle):
    return 500 + (clamp_angle(angle) / 180.0) * 2000


def duty_percent(angle):
    return pulse_us(angle) / 20000.0 * 100.0


def duty_u16(angle):
    return int((pulse_us(angle) / 20000.0) * 65535)


def pose_with(j, angle):
    p = list(REST_POSE)
    p[j] = angle
    return p


def choose_joint():
    for i, n in enumerate(JOINTS, 1):
        print(f"  {i}  {n} ({PICO_PINS[i - 1]})")
    while True:
        s = ask("Joint number: ").strip().lower()
        if s in ("q", "quit"):
            raise StopTest()
        if s in ("1", "2", "3", "4"):
            return int(s) - 1


class Recorder:
    """Appends rows to a CSV and flushes every row so Ctrl+C loses nothing."""

    def __init__(self, path, header):
        self.path = path
        new = not os.path.exists(path)
        self.fh = open(path, "a", newline="")
        self.w = csv.writer(self.fh)
        if new:
            self.w.writerow(header)
            self.fh.flush()

    def row(self, *vals):
        self.w.writerow(vals)
        self.fh.flush()

    def close(self):
        self.fh.close()


# ---------------------------------------------------------------------------
# Link to the Pico (real serial, or a modelled Pico for dry runs)
# ---------------------------------------------------------------------------
class MoveResult:
    def __init__(self, ok, elapsed, logs):
        self.ok = ok
        self.elapsed = elapsed
        self.logs = logs


class Link:
    def __init__(self, port, baud, sim):
        self.sim = sim
        self.ser = None
        self._buf = b""
        self._sim_pose = [0.0] * 4
        if not sim:
            self.ser = serial.Serial(port, baud, timeout=0.02)
            time.sleep(2.0)     # let the port settle
            self.ser.reset_input_buffer()

    # ---- raw line reading with a buffer so partial lines are never lost
    def _lines(self):
        data = self.ser.read(self.ser.in_waiting or 1)
        if data:
            self._buf += data
        out = []
        while b"\n" in self._buf:
            line, self._buf = self._buf.split(b"\n", 1)
            text = line.decode("utf-8", "replace").strip()
            if text:
                out.append(text)
        return out

    @staticmethod
    def _format(angles):
        return ",".join(f"{clamp_angle(a):.2f}" for a in angles) + "\n"

    def move(self, angles, timeout=MOVE_TIMEOUT_S):
        """Send 4 angles, wait for DONE. Returns MoveResult (time = write to DONE)."""
        if self.sim:
            return self._sim_move(angles)
        self.ser.reset_input_buffer()
        self._buf = b""
        t0 = time.perf_counter()
        self.ser.write(self._format(angles).encode("utf-8"))
        logs = []
        while time.perf_counter() - t0 < timeout:
            for text in self._lines():
                if text == "DONE":
                    return MoveResult(True, time.perf_counter() - t0, logs)
                logs.append(text)
        return MoveResult(False, None, logs)

    def exchange(self, text, wait_s):
        """Send one raw line and collect whatever the Pico prints for wait_s."""
        if self.sim:
            return self._sim_exchange(text)
        self.ser.reset_input_buffer()
        self._buf = b""
        self.ser.write((text + "\n").encode("utf-8"))
        lines = []
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < wait_s:
            lines.extend(self._lines())
        return lines

    # ---- modelled Pico (mirrors the firmware logic; NOT a measurement)
    def _sim_move(self, angles):
        new = [clamp_angle(a) for a in angles]
        ticks = max(math.ceil(abs(n - c) / s) for n, c, s in zip(new, self._sim_pose, MAX_STEP))
        wait = (ticks + 0.5) * TICK_S     # half a tick for the command to be picked up
        time.sleep(wait)
        self._sim_pose = new
        return MoveResult(True, wait, [])

    def _sim_exchange(self, text):
        try:
            parts = [float(v) for v in text.split(",")]
        except ValueError:
            time.sleep(0.05)
            return ["LOG: Invalid serial command format."]
        time.sleep(0.05)
        return []

    def close(self):
        if self.ser is not None:
            self.ser.close()


class Ctx:
    def __init__(self, link, out_dir):
        self.link = link
        self.out = out_dir

    def path(self, n, name=None, ext="csv"):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        tag = "_SIM" if self.link.sim else ""
        label = name or f"test{n}"
        return os.path.join(self.out, f"{stamp}_{label}{tag}.{ext}")


# ---------------------------------------------------------------------------
# TEST 1 - serial protocol
# ---------------------------------------------------------------------------
def test_serial(ctx):
    link = ctx.link
    rec = Recorder(ctx.path(1), ["check", "result", "detail"])
    rows = []

    def record(check, ok, detail=""):
        rows.append((check, "PASS" if ok else "FAIL", detail))
        rec.row(check, "PASS" if ok else "FAIL", detail)
        print(f"  [{'PASS' if ok else 'FAIL'}] {check}  {detail}")

    try:
        print("Step 1: move to HOME and wait for DONE ...")
        r = link.move(HOME)
        record("Move command returns DONE", r.ok, f"{r.elapsed:.3f} s" if r.ok else "timed out")
        if not r.ok:
            print("  No DONE received. Check the port, that the Pico is running main.py,")
            print("  and that Thonny is not holding the port open.")
            return

        print("Step 2: DONE latency when the arm does not need to move (10 repeats) ...")
        lat = []
        for _ in range(10):
            r = link.move(HOME, timeout=2.0)
            if r.ok:
                lat.append(r.elapsed * 1000)
            time.sleep(0.05)
        m, sd, n = mean_sd(lat)
        record("No-motion command returns DONE (10/10)", n == 10,
               f"{n}/10, mean {f(m, 1)} ms, min {f(min(lat) if lat else None, 1)}, "
               f"max {f(max(lat) if lat else None, 1)} ms")

        print("Step 3: non-numeric command should give a LOG: reply ...")
        lines = link.exchange("abc,1,2,3", 0.6)
        record("Non-numeric line -> LOG: message", any(l.startswith("LOG:") for l in lines),
               "; ".join(lines) if lines else "no reply")

        print("Step 4: wrong number of values should be ignored (no reply) ...")
        lines = link.exchange("1,2,3", 0.6)
        record("3-value line is ignored (no reply)", len(lines) == 0,
               "; ".join(lines) if lines else "no reply")

        print("Step 5: Pico still responds after bad input ...")
        r = link.move(HOME, timeout=3.0)
        record("Pico alive after invalid input", r.ok)
    finally:
        rec.close()
    passed = sum(1 for _, res, _ in rows if res == "PASS")
    print(f"\nTest 1 result: {passed}/{len(rows)} checks passed. Saved: {rec.path}")


# ---------------------------------------------------------------------------
# TEST 2 - smoothing speed / timing
# ---------------------------------------------------------------------------
def test_smoothing(ctx):
    link = ctx.link
    rec = Recorder(ctx.path(2), ["joint", "span_deg", "direction", "repeat",
                                 "ticks", "nominal_s", "measured_s"])
    summary = []
    try:
        print("Nominal time for a move = ceil(span / max_step) ticks x 20 ms.")
        print("Measured time also includes the 0-20 ms wait for the next tick and USB delay,")
        print("so two span sizes are used to separate that fixed offset from the tick length.\n")
        for j, name in enumerate(JOINTS):
            lo, hi = LIMITS[j]
            spans = [s for s in SPANS if hi - lo >= s]
            if len(spans) < 2:
                print(f"{name}: LIMITS range too small for two spans {SPANS}, skipped.")
                continue
            print(f"--- {name} ({PICO_PINS[j]}), range {lo}-{hi} deg, step {MAX_STEP[j]} deg/tick ---")
            per_span = {}
            for span in spans:
                a, b = lo, lo + span
                pa, pb = pose_with(j, a), pose_with(j, b)
                link.move(pa)               # go to the start (not timed)
                time.sleep(0.2)
                ticks = math.ceil(span / MAX_STEP[j])
                times = []
                for rep in range(1, REPEATS + 1):
                    for direction, target in (("up", pb), ("down", pa)):
                        r = link.move(target)
                        if not r.ok:
                            print(f"  no DONE for {name} {direction} {span} deg, stopping this test.")
                            return
                        times.append(r.elapsed)
                        rec.row(name, span, direction, rep, ticks,
                                f"{ticks * TICK_S:.3f}", f"{r.elapsed:.4f}")
                        time.sleep(0.1)
                m, sd, _ = mean_sd(times)
                per_span[span] = (ticks, m, sd)
                print(f"  {span:>3} deg: {ticks:>3} ticks, nominal {ticks * TICK_S:.3f} s, "
                      f"measured {m:.3f} +/- {sd:.3f} s")
            (s1, s2) = spans[0], spans[1]
            t1, m1, _ = per_span[s1]
            t2, m2, _ = per_span[s2]
            eff_tick = (m2 - m1) / (t2 - t1)
            offset = m1 - t1 * eff_tick
            speed_nom = MAX_STEP[j] / TICK_S
            speed_eff = MAX_STEP[j] / eff_tick
            full = math.ceil(180 / MAX_STEP[j]) * eff_tick + offset
            summary.append((name, speed_nom, speed_eff, eff_tick * 1000, offset * 1000,
                            math.ceil(180 / MAX_STEP[j]) * TICK_S, full))
            print(f"  effective tick {eff_tick * 1000:.1f} ms (nominal 20), fixed offset {offset * 1000:.0f} ms\n")
    finally:
        rec.close()
    if summary:
        print("\nSummary (compare with Table 1 in the report)")
        print(f"{'Joint':<9}{'nom deg/s':>10}{'meas deg/s':>12}{'tick ms':>9}{'offset ms':>11}"
              f"{'180 nom s':>11}{'180 meas s':>12}")
        for name, sn, se, tk, off, tn, tf in summary:
            print(f"{name:<9}{sn:>10.0f}{se:>12.0f}{tk:>9.1f}{off:>11.0f}{tn:>11.2f}{tf:>12.2f}")
        if link.sim:
            print("\nSIMULATION: these numbers come from the model, not from the arm.")
        print(f"\nSaved: {rec.path}")


# ---------------------------------------------------------------------------
# TEST 3 - commanded vs measured angle (+ duty)
# ---------------------------------------------------------------------------
def test_angle_sweep(ctx):
    link = ctx.link
    j = choose_joint()
    name = JOINTS[j]
    lo, hi = LIMITS[j]
    pts = [a for a in ANGLE_POINTS if lo <= a <= hi]
    if len(pts) < 3:
        print("Fewer than 3 test angles fit inside LIMITS for this joint.")
        return
    sweep = [("up", a) for a in pts] + [("down", a) for a in reversed(pts[:-1])]
    print(f"\nSignal pin for {name}: {PICO_PINS[j]}. Measure duty with the meter's % mode (or pulse width")
    print("with a scope) and the joint angle with a protractor or phone inclinometer.")
    print("Press Enter to skip any value, q to stop.\n")
    rec = Recorder(ctx.path(3, f"test3_{name}"),
                   ["joint", "direction", "commanded_deg", "expected_duty_pct", "expected_duty_u16",
                    "measured_duty_pct", "measured_angle_deg"])
    results = []
    try:
        link.move(pose_with(j, sweep[0][1]))
        for direction, a in sweep:
            r = link.move(pose_with(j, a))
            if not r.ok:
                print("  no DONE, stopping.")
                return
            time.sleep(SETTLE_S)
            print(f"{name} commanded {a} deg ({direction}) | expected duty {duty_percent(a):.2f} % "
                  f"(pulse {pulse_us(a):.0f} us, duty_u16 {duty_u16(a)})")
            ma = ask_float("  measured angle (deg): ")
            md = ask_float("  measured duty (%): ")
            rec.row(name, direction, a, f"{duty_percent(a):.3f}", duty_u16(a), md, ma)
            results.append((direction, a, ma, md))
    finally:
        rec.close()

    print(f"\n{name}: up vs down (hysteresis = up - down)")
    print(f"{'cmd':>5}{'up':>9}{'down':>9}{'diff':>8}")
    for a in pts:
        up = next((m for d, c, m, _ in results if d == "up" and c == a), None)
        dn = next((m for d, c, m, _ in results if d == "down" and c == a), None)
        diff = None if up is None or dn is None else up - dn
        print(f"{a:>5}{f(up, 1):>9}{f(dn, 1):>9}{f(diff, 1):>8}")
    for label, idx in (("measured angle", 2), ("measured duty", 3)):
        xs = [c for d, c, *rest in results if rest[idx - 2] is not None]
        ys = [rest[idx - 2] for d, c, *rest in results if rest[idx - 2] is not None]
        if len(xs) >= 3 and len(set(xs)) > 1:
            fit = statistics.linear_regression(xs, ys)
            print(f"Linear fit of {label} vs commanded: slope {fit.slope:.4f}, intercept {fit.intercept:.3f}")
    print("Ideal servo: angle slope 1.000, intercept 0, up = down. Duty slope 0.0556 %/deg, intercept 2.5 %.")
    print(f"Saved: {rec.path}")


# ---------------------------------------------------------------------------
# TEST 4 - held-pose multimeter readings
# ---------------------------------------------------------------------------
def test_meter(ctx):
    link = ctx.link
    print("Set the meter to DC current (10 A input, IN SERIES with the supply line, never across it).")
    where = ask("Where is the meter connected? (label, e.g. 'total 5 V line' or 'shoulder servo VCC'): ").strip()
    rec = Recorder(ctx.path(4), ["meter_location", "state", "reading", "current_mA", "rail_V", "peak_mA"])
    table = []
    try:
        for name, pose, note in CURRENT_STATES:
            print(f"\n=== State: {name} ===")
            if note:
                print(f"  {note}")
            s = ask("  Set this up, then press Enter (or 's' to skip this state): ").strip().lower()
            if s == "s":
                continue
            peak = None
            if pose is not None:
                ask("  Reset the meter's MAX hold, then press Enter to MOVE the arm: ")
                r = link.move(pose)
                if not r.ok:
                    print("  no DONE received, stopping.")
                    return
                peak = ask_float("  Peak current during the move (MAX hold, mA, Enter to skip): ")
                print(f"  Letting the current settle for {HOLD_SETTLE_S:.0f} s ...")
                time.sleep(HOLD_SETTLE_S)
            amps, volts = [], []
            for k in range(1, READINGS + 1):
                c = ask_float(f"  Reading {k}/{READINGS} current (mA, Enter to finish this state): ")
                if c is None:
                    break
                v = ask_float("    rail voltage at the servo (V, Enter to skip): ")
                amps.append(c)
                volts.append(v)
                rec.row(where, name, k, c, v, peak)
            table.append((name, mean_sd(amps), mean_sd(volts), peak))
    finally:
        rec.close()
    if table:
        print(f"\nSummary (meter: {where})")
        print(f"{'State':<36}{'I mean mA':>10}{'SD':>7}{'n':>3}{'V mean':>8}{'peak mA':>9}")
        for name, (im, isd, n), (vm, _, _), peak in table:
            print(f"{name:<36}{f(im, 1):>10}{f(isd, 1):>7}{n:>3}{f(vm, 2):>8}{f(peak, 0):>9}")
        print("Ideal vs active: compare 'Pico only' and unloaded readings with the loaded poses.")
    print(f"Saved: {rec.path}")


# ---------------------------------------------------------------------------
# TEST 5 - IK / FK round trip and base mapping (no arm)
# ---------------------------------------------------------------------------
def test_ik(ctx):
    if ik is None:
        print("IKtest.py could not be imported. Run this from the folder that contains it.")
        return
    table_z = 0.0
    if hsv is not None:
        try:
            _, table_z = hsv.load_calibration()
        except Exception as e:
            print(f"(calibration.json not loaded: {e}; using table z = 0)")
    zs = [table_z, table_z + 4.0]
    print(f"Targets: x {IK_X}, y {IK_Y}, z {zs} cm (table z and the +4 cm hover pose).")
    print("Link lengths in IKtest: L0 =", ik.L0, " L1 =", ik.L1, " L2 =", ik.L2, "cm\n")
    rec = Recorder(ctx.path(5), ["x", "y", "z", "status", "base", "shoulder", "elbow",
                                 "fk_x", "fk_y", "fk_z", "fk_error_cm"])
    n_total = n_ok = n_clamp = 0
    clean_err = []
    print(f"{'x':>5}{'y':>5}{'z':>6}  {'base':>6}{'shldr':>7}{'elbow':>7}  {'FK err':>8}  status")
    try:
        for z in zs:
            for x in IK_X:
                for y in IK_Y:
                    n_total += 1
                    try:
                        b, s, e = ik.solver(x, y, z)
                    except ValueError:
                        rec.row(x, y, z, "UNREACHABLE", "", "", "", "", "", "", "")
                        print(f"{x:>5}{y:>5}{z:>6.1f}  {'':>6}{'':>7}{'':>7}  {'':>8}  unreachable")
                        continue
                    fx, fy, fz = ik.fk(b, s, e)
                    err = math.sqrt((fx - x) ** 2 + (fy - y) ** 2 + (fz - z) ** 2)
                    clamped = any(v <= 0.0 or v >= 180.0 for v in (b, s, e))
                    status = "CLAMPED" if clamped else "ok"
                    n_ok += 1
                    if clamped:
                        n_clamp += 1
                    else:
                        clean_err.append(err)
                    rec.row(x, y, z, status, f"{b:.2f}", f"{s:.2f}", f"{e:.2f}",
                            f"{fx:.3f}", f"{fy:.3f}", f"{fz:.3f}", f"{err:.4f}")
                    print(f"{x:>5}{y:>5}{z:>6.1f}  {b:>6.1f}{s:>7.1f}{e:>7.1f}  {err:>8.3f}  {status}")
    finally:
        rec.close()
    print(f"\n{n_total} targets: {n_total - n_ok} unreachable, {n_ok} solved, {n_clamp} of those clamped.")
    if clean_err:
        print(f"Unclamped FK round-trip error: mean {statistics.mean(clean_err):.4f} cm, "
              f"max {max(clean_err):.4f} cm  (should be ~0: it only checks the maths)")
    print("CLAMPED rows have a joint command pinned at 0 or 180, so the arm would not reach the target.")
    print("Base rule: command = 90 + atan2(y, x). x < 0 (behind the base) cannot be reached.")
    print(f"Saved: {rec.path}")


# ---------------------------------------------------------------------------
# TEST 6 - vision: detected vs measured
# ---------------------------------------------------------------------------
def test_vision(ctx):
    if cv2 is None or hsv is None:
        print("OpenCV or hsvtest.py could not be imported.")
        return
    try:
        H, table_z = hsv.load_calibration()
    except Exception as e:
        print(f"Could not load calibration.json: {e}")
        return
    frame = None
    if confirm("Capture a new image from the webcam (otherwise detect_photo.jpg is used)?"):
        idx = ask("Camera index (Enter = 0): ").strip()
        cap = cv2.VideoCapture(int(idx) if idx else 0)
        if not cap.isOpened():
            print("Could not open the camera.")
            return
        for _ in range(15):
            cap.read()          # let exposure settle
        ok, frame = cap.read()
        cap.release()
        if not ok:
            print("Could not read a frame.")
            return
        cv2.imwrite(ctx.path(6, "capture", "jpg"), frame)
    else:
        frame = cv2.imread(hsv.IMAGE_PATH)
        if frame is None:
            print(f"Could not read {hsv.IMAGE_PATH}")
            return
    found = hsv.find_blocks(frame, H, table_z)
    if not found:
        print("No blocks detected.")
        return
    annotated = frame.copy()
    row = 25
    print("\nDetected (cm from the base axis):")
    for color, (x, y, z) in found.items():
        print(f"  {color:<7} x={x:6.2f}  y={y:6.2f}  z={z:5.2f}")
        cv2.putText(annotated, f"{color}: x={x:.1f} y={y:.1f}", (10, row),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, hsv.DRAW_COLORS.get(color, (255, 255, 255)), 2)
        row += 22
    img_path = ctx.path(6, "annotated", "jpg")
    cv2.imwrite(img_path, annotated)
    print(f"Annotated image saved: {img_path}")
    print("\nMeasure each block's real position with a ruler (centre of the base edge on the table).")
    rec = Recorder(ctx.path(6), ["color", "det_x", "det_y", "true_x", "true_y", "dx", "dy", "dist_cm"])
    errs = []
    try:
        for color, (x, y, z) in found.items():
            print(f"{color}: detected x={x:.2f}, y={y:.2f}")
            tx = ask_float("  true x (cm, Enter to skip): ")
            ty = ask_float("  true y (cm, Enter to skip): ")
            if tx is None or ty is None:
                rec.row(color, f"{x:.3f}", f"{y:.3f}", tx, ty, "", "", "")
                continue
            dx, dy = x - tx, y - ty
            d = math.hypot(dx, dy)
            errs.append(d)
            rec.row(color, f"{x:.3f}", f"{y:.3f}", tx, ty, f"{dx:.3f}", f"{dy:.3f}", f"{d:.3f}")
            print(f"  error dx={dx:+.2f}, dy={dy:+.2f}, distance {d:.2f} cm")
    finally:
        rec.close()
    if errs:
        print(f"\nPosition error: mean {statistics.mean(errs):.2f} cm, max {max(errs):.2f} cm over {len(errs)} block(s).")
    print(f"Saved: {rec.path}")


# ---------------------------------------------------------------------------
# TEST 7 - all four joints at once: current vs distance
# ---------------------------------------------------------------------------
def test_current_distance(ctx):
    link = ctx.link
    steps = []
    for frac in CURRENT_FRACTIONS:
        pose = [lo + frac * (hi - lo) for lo, hi in LIMITS]
        moves = [abs(p - h) for p, h in zip(pose, HOME)]
        ticks = max(math.ceil(m / s - 1e-9) for m, s in zip(moves, MAX_STEP))
        steps.append((frac, pose, max(moves), ticks))

    print("All four joints are commanded at once (a single 4-angle command), out and then back to 0.")
    print("Each joint has its own max_step, so they arrive at different times and the slowest decides.\n")
    print(f"{'step':>6}{'Claw':>8}{'Elbow':>8}{'Shoulder':>10}{'Base':>7}{'largest move':>14}{'ticks':>7}{'nominal s':>11}")
    for frac, pose, largest, ticks in steps:
        print(f"{frac:>6.0%}{pose[0]:>8.0f}{pose[1]:>8.0f}{pose[2]:>10.0f}{pose[3]:>7.0f}"
              f"{largest:>12.0f} deg{ticks:>7}{ticks * TICK_S:>11.2f}")
    if not confirm("\nAre ALL of these poses safe for the arm (no collisions, claw will not stall)?"):
        print("Cancelled. Edit LIMITS or CURRENT_FRACTIONS at the top of this file.")
        return

    print("\nSet the meter to DC current (10 A input, IN SERIES with the supply line, never across it).")
    print("Use its MAX / peak hold if it has one. For each move you reset it first, then read it after.")
    where = ask("Where is the meter connected? (e.g. 'total 5 V line'): ").strip()
    rec = Recorder(ctx.path(7), ["meter_location", "step_fraction", "phase", "reading", "current_mA",
                                 "rail_V", "peak_mA", "move_time_s", "largest_move_deg", "ticks"])
    table = []

    def read_hold(frac, phase, peak, move_s, largest, ticks):
        amps, volts = [], []
        print(f"  Letting the current settle for {HOLD_SETTLE_S:.0f} s ...")
        time.sleep(HOLD_SETTLE_S)
        for k in range(1, READINGS + 1):
            c = ask_float(f"  Reading {k}/{READINGS} holding current (mA, Enter to finish): ")
            if c is None:
                break
            v = ask_float("    rail voltage at the servo (V, Enter to skip): ")
            amps.append(c)
            volts.append(v)
            rec.row(where, frac, phase, k, c, v, peak, move_s, largest, ticks)
        return mean_sd(amps)[0]

    try:
        print("\n=== Baseline: arm holding HOME ===")
        if not link.move(HOME).ok:
            print("  no DONE received, stopping.")
            return
        base_mean = read_hold(0, "home_baseline", None, None, 0, 0)

        for frac, pose, largest, ticks in steps:
            print(f"\n=== Step {frac:.0%}: all joints to {[round(p) for p in pose]} "
                  f"(largest move {largest:.0f} deg, {ticks} ticks) ===")
            s = ask("  Reset the meter's MAX hold, then press Enter to MOVE OUT (s = skip step): ").strip().lower()
            if s == "s":
                continue
            r = link.move(pose)
            if not r.ok:
                print("  no DONE received, stopping.")
                return
            out_s = r.elapsed
            print(f"  move out took {out_s:.2f} s")
            peak_out = ask_float("  Peak current during the move out (MAX hold, mA, Enter to skip): ")
            rec.row(where, frac, "move_out_peak", "", "", "", peak_out, f"{out_s:.3f}", f"{largest:.0f}", ticks)
            hold_out = read_hold(frac, "hold_at_pose", peak_out, f"{out_s:.3f}", f"{largest:.0f}", ticks)

            ask("  Reset the meter's MAX hold, then press Enter to MOVE BACK to 0: ")
            r = link.move(HOME)
            if not r.ok:
                print("  no DONE received, stopping.")
                return
            back_s = r.elapsed
            print(f"  move back took {back_s:.2f} s")
            peak_back = ask_float("  Peak current during the move back (MAX hold, mA, Enter to skip): ")
            rec.row(where, frac, "move_back_peak", "", "", "", peak_back, f"{back_s:.3f}", f"{largest:.0f}", ticks)
            hold_zero = read_hold(frac, "hold_at_home", peak_back, f"{back_s:.3f}", f"{largest:.0f}", ticks)
            table.append((frac, largest, ticks, out_s, peak_out, hold_out, peak_back, hold_zero))
    finally:
        rec.close()

    if table:
        print(f"\nSummary (meter: {where}); baseline holding HOME before any move: {f(base_mean, 1)} mA")
        print(f"{'step':>6}{'largest deg':>12}{'ticks':>7}{'move s':>8}{'peak out':>10}{'hold@pose':>11}"
              f"{'peak back':>11}{'hold@0':>8}   (currents in mA)")
        for frac, largest, ticks, t, po, ho, pb, hz in table:
            print(f"{frac:>6.0%}{largest:>12.0f}{ticks:>7}{t:>8.2f}{f(po, 0):>10}{f(ho, 0):>11}{f(pb, 0):>11}{f(hz, 0):>8}")
        if link.sim:
            print("\nSIMULATION: the move times are modelled; the currents are whatever you typed.")
        print("\nWhat to look for:")
        print("  - Peak vs distance: the firmware limits each joint to a constant speed, so the peak is expected")
        print("    to depend mainly on how many joints are moving at once, while the DURATION grows with distance.")
        print("  - hold@pose minus the HOME baseline is the extra holding current from gravity and the claw.")
        print("  - Joints finish at different times (Claw first, Shoulder/Base last), so the current steps down")
        print("    during a move; a multimeter only shows the peak. A current sensor logging over time is needed")
        print("    for the real curve.")
    print(f"Saved: {rec.path}")


# ---------------------------------------------------------------------------
# Menu
# ---------------------------------------------------------------------------
TESTS = {
    "1": ("Serial protocol check", test_serial, True,
          "DONE handshake, DONE latency, invalid and wrong-length commands.",
          "Pico running main.py, USB connected. The arm only goes to HOME (all 0)."),
    "2": ("Smoothing speed / timing", test_smoothing, True,
          "Move time per joint versus ceil(span / max_step) x 20 ms (Table 1).",
          "The arm moves each joint within LIMITS, about 12 moves per joint."),
    "3": ("Commanded vs measured angle (+ duty)", test_angle_sweep, True,
          "Angle linearity and up/down hysteresis of one joint, and the duty-cycle mapping.",
          "Protractor or phone inclinometer; multimeter in duty (%) mode on the signal pin."),
    "4": ("Held-pose meter readings", test_meter, True,
          "Current and rail voltage in each state (ideal vs active load, peak current).",
          "Multimeter on the 10 A input in series with the supply line. Edit CURRENT_STATES first."),
    "5": ("IK / FK round trip and base mapping", test_ik, False,
          "FK error over a grid of targets, unreachable and clamped targets (x < 0).",
          "No arm needed."),
    "6": ("Vision: detected vs measured position", test_vision, False,
          "Block positions from the webcam and HSV pipeline versus ruler measurements.",
          "Blocks placed on the sheet, ruler, webcam in its final position."),
    "7": ("All joints at once: current vs distance", test_current_distance, True,
          "Current while all four joints move together out and back to 0, at several distances.",
          "Multimeter (10 A input, in series with the supply line, MAX hold if available). Check LIMITS and CURRENT_FRACTIONS."),
}


def show_menu(link):
    mode = "SIMULATION (no arm, modelled timing)" if link.sim else "REAL ARM"
    print("\n" + "=" * 62)
    print(f" 4-DOF arm test menu   [{mode}]")
    print("=" * 62)
    for k, (title, _, needs_arm, _, _) in TESTS.items():
        print(f"  {k}  {title}" + ("" if needs_arm else "   (no arm movement)"))
    print("  h  Send HOME pose (all 0)")
    print("  q  Quit")


def run_test(key, ctx):
    title, func, moves, checks, prep = TESTS[key]
    print(f"\n--- Test {key}: {title} ---")
    print(f"Checks : {checks}")
    print(f"Before : {prep}")
    if ctx.link.sim and key in ("1", "2", "3", "4", "7"):
        print("NOTE   : SIMULATION - results are modelled, not measured.")
    if key in ("2", "3", "4", "7") and not ctx.link.sim:
        print(f"LIMITS in use (deg): {dict(zip(JOINTS, LIMITS))}")
    try:
        if moves and not ctx.link.sim:
            if not confirm("Is the workspace clear and your hand ready at the power switch?"):
                print("Cancelled.")
                return
        s = ask(f"Press Enter to START test {key} (or type b to go back): ").strip().lower()
        if s == "b":
            return
        func(ctx)
    except StopTest:
        print("\nTest stopped.")
    except KeyboardInterrupt:
        print("\nStopped with Ctrl+C. The arm holds its last pose.")
    except Exception as e:      # keep the menu alive whatever happens
        print(f"\nTest failed with an error: {type(e).__name__}: {e}")
    print(f"\nTest {key} finished. Nothing else will start by itself.")
    try:
        input("Press Enter to return to the menu ... ")
    except EOFError:
        pass


def main():
    ap = argparse.ArgumentParser(description="PC-side tests for the 4-DOF arm")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--sim", action="store_true", help="dry run with a modelled Pico")
    ap.add_argument("--out", default="test_results")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    sim = args.sim
    link = None
    if not sim:
        if serial is None:
            print("pyserial is not installed (pip install pyserial).")
            sim = True
        else:
            try:
                link = Link(args.port, args.baud, False)
                print(f"Connected to {args.port} at {args.baud} baud.")
            except serial.SerialException as e:
                print(f"Could not open {args.port}: {e}")
                sim = True
        if sim and not confirm("Continue in SIMULATION mode instead?"):
            return
    if sim:
        link = Link(args.port, args.baud, True)
        print("SIMULATION mode: no arm is connected, timings are modelled.")
    ctx = Ctx(link, args.out)

    try:
        while True:
            show_menu(link)
            try:
                choice = input("Choose a test: ").strip().lower()
            except EOFError:
                break
            if choice in ("q", "quit"):
                break
            if choice == "h":
                if link.sim or confirm("Is the workspace clear? Move all joints to 0"):
                    r = link.move(HOME)
                    print("HOME reached." if r.ok else "No DONE received.")
            elif choice in TESTS:
                run_test(choice, ctx)
            else:
                print("Pick 1-7, h or q.")
    except KeyboardInterrupt:
        print("\nExiting. The arm holds its last pose.")
    finally:
        link.close()


if __name__ == "__main__":
    main()