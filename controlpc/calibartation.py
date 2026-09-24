"""
Image-based Camera -> Robot-frame calibration (planar homography)
=====================================================================
Uses a single saved photo instead of a live webcam feed. Take one photo
with your camera in its FINAL mounted position (phone photo held in the
same spot is fine for now, but ideally the actual camera you'll run
detection with later), save it, and point IMAGE_PATH at it.

PHYSICAL SETUP: mark 4+ points on the table, measure their (x, y) from
the base rotation axis with a tape measure (x=forward, y=left), and make
sure ALL of them are visible in the photo before you take it.

RUNNING IT
-----------
- Set IMAGE_PATH below to your photo's file path.
- Enter the table height when prompted (once).
- Click each marked point in the image window; type in the (x, y) you
  measured for that point when prompted in the terminal.
- Press 'c' once you've clicked 4+ points to compute and save
  calibration.json.
- Press 'q' to quit at any time.
"""

import cv2
import numpy as np
import json
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
IMAGE_PATH = str(SCRIPT_DIR / "calibration_photo.jpg")   # <-- point this at your saved photo
MIN_POINTS = 4
MAX_POINTS = 8
SAVE_PATH = str(SCRIPT_DIR / "calibration.json")

pixel_points = []
world_points = []
homography = None


def collect_point_callback(event, u, v, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and homography is None and len(pixel_points) < MAX_POINTS:
        print(f"\nClicked pixel ({u}, {v}).")
        try:
            x = float(input("  Measured x (forward, cm): "))
            y = float(input("  Measured y (left, cm): "))
        except ValueError:
            print("  Not a number -- point discarded, click it again.")
            return
        pixel_points.append([u, v])
        world_points.append([x, y])
        print(f"  Recorded point {len(pixel_points)}/{MAX_POINTS}.")


def test_point_callback(event, u, v, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and homography is not None:
        pt = np.array([[[u, v]]], dtype=np.float32)
        world = cv2.perspectiveTransform(pt, homography)
        x, y = world[0][0]
        print(f"pixel ({u},{v}) -> world x={x:.2f}cm, y={y:.2f}cm")


def _pixel_to_world_preview(u, v, H, table_z_cm):
    pt = np.array([[[u, v]]], dtype=np.float32)
    world = cv2.perspectiveTransform(pt, H)
    x, y = world[0][0]
    return float(x), float(y), table_z_cm


def main():
    global homography

    base_img = cv2.imread(IMAGE_PATH)
    if base_img is None:
        raise FileNotFoundError(f"Could not read image at {IMAGE_PATH}")

    table_z_cm = float(input("Table height relative to base axis, cm (negative if below): "))

    cv2.namedWindow("Calibration")
    cv2.setMouseCallback("Calibration", collect_point_callback)

    print(f"\nClick your {MIN_POINTS}+ marked points in the image window.")
    print("Press 'c' when done clicking to compute the homography. Press 'q' to quit.\n")

    while True:
        frame = base_img.copy()
        for (u, v) in pixel_points:
            cv2.circle(frame, (u, v), 6, (0, 255, 0), 2)

        status = f"Points: {len(pixel_points)}/{MIN_POINTS}+  (press 'c' to compute, 'q' to quit)"
        cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("Calibration", frame)

        key = cv2.waitKey(50) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
            return
        if key == ord('c') and len(pixel_points) >= MIN_POINTS:
            break
        if len(pixel_points) >= MAX_POINTS:
            break

    px = np.array(pixel_points, dtype=np.float32)
    wp = np.array(world_points, dtype=np.float32)
    homography, status = cv2.findHomography(px, wp, cv2.RANSAC)
    if homography is None:
        print("Homography failed -- check your points aren't all in a line, then rerun.")
        cv2.destroyAllWindows()
        return

    with open(SAVE_PATH, "w") as f:
        json.dump({"H": homography.tolist(), "table_z_cm": table_z_cm}, f, indent=2)
    print(f"\nSaved calibration to {SAVE_PATH}")

    print("\n--- Self-check (should be close to what you measured) ---")
    for (u, v), (wx, wy) in zip(pixel_points, world_points):
        x, y, z = _pixel_to_world_preview(u, v, homography, table_z_cm)
        print(f"pixel ({u},{v}) -> world ({x:.2f},{y:.2f})  [expected ({wx:.1f},{wy:.1f})]")

    print("\nTEST MODE -- click any point in the image to see its world coordinates.")
    print("Best check: click a point you did NOT use for calibration and compare to a tape measure.")
    cv2.setMouseCallback("Calibration", test_point_callback)

    while True:
        frame = base_img.copy()
        cv2.putText(frame, "TEST MODE -- click to check, 'q' to quit", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        cv2.imshow("Calibration", frame)
        if cv2.waitKey(50) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()