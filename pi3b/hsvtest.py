"""
Locate colored blocks in the robot's world frame using webcam + saved calibration.
=====================================================================================
Loads calibration.json (produced by camera_calibration.py -- run that first,
separately, any time the camera moves or you recalibrate). Returns the
real-world (x, y, z) position of the largest block of each color, using the
bottom-center of its bounding box as the contact point on the table.

Run this file directly for a live preview with all 4 colors detected and
labeled on screen. Import find_blocks() / find_block() to use it from other
code, e.g. main.py.
"""

import cv2
import numpy as np
import json

import cv2
import numpy as np
import json
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
CALIBRATION_PATH = str(SCRIPT_DIR / "calibration.json")
IMAGE_PATH = str(SCRIPT_DIR / "detect_photo.jpg")   # <-- point this at the photo you want to check

COLOR_RANGES = {
    "Red": [
        (np.array([0, 130, 90]), np.array([10, 255, 255])),
        (np.array([170, 130, 90]), np.array([179, 255, 255])),
    ],
    "Blue": [(np.array([95, 120, 80]), np.array([130, 255, 255]))],
    "Green": [(np.array([45, 80, 60]), np.array([85, 255, 255]))],
    "Yellow": [(np.array([20, 120, 120]), np.array([35, 255, 255]))],
}

# BGR colors for drawing each detection's box in the preview window
DRAW_COLORS = {
    "Red": (0, 0, 255),
    "Blue": (255, 0, 0),
    "Green": (0, 255, 0),
    "Yellow": (0, 255, 255),
}

MIN_CONTOUR_AREA = 200


def load_calibration(path=CALIBRATION_PATH):
    """Returns (H, table_z_cm) loaded from calibration.json."""
    with open(path, "r") as f:
        data = json.load(f)
    H = np.array(data["H"], dtype=np.float32)
    table_z_cm = float(data["table_z_cm"])
    return H, table_z_cm


def pixel_to_world(u, v, H, table_z_cm):
    pt = np.array([[[u, v]]], dtype=np.float32)
    world = cv2.perspectiveTransform(pt, H)
    x, y = world[0][0]
    return float(x), float(y), table_z_cm


def _largest_box_for_mask(mask):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < MIN_CONTOUR_AREA:
        return None
    return cv2.boundingRect(largest)  # (x, y, w, h)


def find_blocks(frame, H, table_z_cm, color_ranges=COLOR_RANGES):
    """
    Given a single BGR frame, return {color_name: (x, y, z)} for every color
    with a block found large enough. Colors with nothing detected are omitted
    from the result -- check with `"Red" in results` rather than indexing directly.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    results = {}

    for color_name, ranges in color_ranges.items():
        mask = None
        for lower, upper in ranges:
            m = cv2.inRange(hsv, lower, upper)
            mask = m if mask is None else (mask | m)

        box = _largest_box_for_mask(mask)
        if box is None:
            continue

        x, y, w, h = box
        contact_u = x + w / 2.0
        contact_v = y + h  # bottom edge -- the point touching the table
        results[color_name] = pixel_to_world(contact_u, contact_v, H, table_z_cm)

    return results


def find_block(frame, H, table_z_cm, color_name):
    """Convenience: look up just one color. Returns (x, y, z) or None."""
    if color_name not in COLOR_RANGES:
        raise ValueError(f"Unknown color '{color_name}'. Options: {list(COLOR_RANGES)}")
    return find_blocks(frame, H, table_z_cm, {color_name: COLOR_RANGES[color_name]}).get(color_name)


if __name__ == "__main__":
    H, table_z_cm = load_calibration()

    frame = cv2.imread(IMAGE_PATH)
    if frame is None:
        raise FileNotFoundError(f"Could not read image at {IMAGE_PATH}")

    results = find_blocks(frame, H, table_z_cm)

    print("--- Detected blocks ---")
    if not results:
        print("  (none found)")
    row = 25
    for color_name, (x, y, z) in results.items():
        print(f"  {color_name}: x={x:.2f} y={y:.2f} z={z:.2f} cm")
        cv2.putText(frame, f"{color_name}: x={x:.1f} y={y:.1f} z={z:.1f} cm",
                    (10, row), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    DRAW_COLORS[color_name], 2)
        row += 22

    cv2.imshow("Block Locator", frame)
    print("\nPress any key on the image window to close.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()