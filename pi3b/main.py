import cv2
import numpy as np
import matplotlib.pyplot as plt

# -----------------------------------------------------------------------
# CALIBRATION PARAMETERS (EASILY INTERCHANGEABLE)
# -----------------------------------------------------------------------
# Real physical width of the cube (0.025 meters = 2.5 cm)
REAL_OBJECT_WIDTH = 0.025  

# Camera Focal Length in pixels (f = (px_width * distance) / real_width)
FOCAL_LENGTH_PX = 600.0  

# -----------------------------------------------------------------------
# ROBOT BASE TRANSFORM CONFIGURATION
# Measure these physical distances (in meters) from your camera lens 
# to the origin of your robot arm's base.
# -----------------------------------------------------------------------
CAMERA_OFFSET_X = 0.00   # Lateral displacement between camera center and robot base
CAMERA_OFFSET_Y = 0.10   # Forward displacement between camera lens and robot base
CAMERA_HEIGHT_Z = 0.35   # Height of the camera lens above the table surface

# -----------------------------------------------------------------------
# SETUP & HSV RANGES
# -----------------------------------------------------------------------
img_path = r"models-pictures\img1.jpg"
img = cv2.imread(img_path)
if img is None:
    raise FileNotFoundError(f"Could not read image at {img_path}")

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
img_h, img_w = img.shape[:2]

# Camera Optical Center
cx, cy = img_w / 2.0, img_h / 2.0

# Pre-calibrated HSV values
masks = {
    "Red": [
        (np.array([0, 130, 90]), np.array([10, 255, 255])),
        (np.array([170, 130, 90]), np.array([179, 255, 255]))
    ],
    "Blue": [(np.array([95, 120, 80]), np.array([130, 255, 255]))],
    "Green": [(np.array([45, 80, 60]), np.array([85, 255, 255]))],
    "Yellow": [(np.array([20, 120, 120]), np.array([35, 255, 255]))],
}

labels = {
    "Red": (0, 0, 255),
    "Blue": (255, 0, 0),
    "Green": (0, 255, 0),
    "Yellow": (0, 255, 255),
}

# Filtering parameters
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
MIN_AREA = 200

# Region of Interest (ROI) mask: ignore top 45% (robotic arm & overhead wires)[cite: 1]
roi_mask = np.zeros((img_h, img_w), dtype=np.uint8)
roi_mask[int(img_h * 0.45):, :] = 255

output = img.copy()

# Store detected targets for IK processing
target_blocks = []

# -----------------------------------------------------------------------
# DETECTION & COORDINATE TRANSFORM
# -----------------------------------------------------------------------
print("\n--- DETECTED OBJECT COORDINATES ---")

for color_name, ranges in masks.items():
    mask = None
    for lower, upper in ranges:
        m = cv2.inRange(hsv, lower, upper)
        mask = m if mask is None else (mask | m)

    mask = cv2.bitwise_and(mask, roi_mask)
    clean_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        if cv2.contourArea(cnt) < MIN_AREA:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = float(w) / h

        if 0.5 <= aspect_ratio <= 2.0:
            # 1. Bounding box center pixel coordinate
            center_x = x + (w / 2.0)
            center_y = y + (h / 2.0)

            # 2. Camera Coordinates (X_c, Y_c, Z_c)
            Z_cam = (FOCAL_LENGTH_PX * REAL_OBJECT_WIDTH) / float(w)
            X_cam = ((center_x - cx) * Z_cam) / FOCAL_LENGTH_PX
            Y_cam = ((center_y - cy) * Z_cam) / FOCAL_LENGTH_PX

            # 3. Transform Camera Frame -> Robot Base Frame (X_b, Y_b, Z_b)
            X_base = X_cam + CAMERA_OFFSET_X
            Y_base = Z_cam + CAMERA_OFFSET_Y       # Depth maps to forward distance
            Z_base = CAMERA_HEIGHT_Z - Y_cam        # Vertical pixel offset maps to table height

            # Save object data
            target_blocks.append({
                "color": color_name,
                "cam_coords": (X_cam, Y_cam, Z_cam),
                "base_coords": (X_base, Y_base, Z_base)
            })

            print(f"[{color_name}] Camera Frame -> X: {X_cam:+.3f}m, Y: {Y_cam:+.3f}m, Z: {Z_cam:.3f}m")
            print(f"       Robot Base   -> X: {X_base:+.3f}m, Y: {Y_base:+.3f}m, Z: {Z_base:.3f}m\n")

            # Visual overlay
            cv2.rectangle(output, (x, y), (x + w, y + h), labels[color_name], 2)
            cv2.circle(output, (int(center_x), int(center_y)), 3, (0, 0, 0), -1)

            overlay_text = f"{color_name} Z:{Z_cam:.2f}m"
            cv2.putText(output, overlay_text, (x, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, labels[color_name], 2)

# -----------------------------------------------------------------------
# INVERSE KINEMATICS (IK) PIPELINE INTEGRATION
# -----------------------------------------------------------------------
def send_to_ik_solver(color, target_x, target_y, target_z):
    """
    Placeholder function for your IK solver / motor controller.
    """
    GRIPPER_PITCH = -90.0  # degrees (pointing straight down)
    APPROACH_Z = target_z + 0.02

    print(f"[IK SOLVER] Planning trajectory for {color} block...")
    print(f"            Target Pose: X={target_x:.3f}m, Y={target_y:.3f}m, Z={target_z:.3f}m")
    print(f"            Approach Z : {APPROACH_Z:.3f}m, Pitch={GRIPPER_PITCH}°")

print("\n--- RUNNING IK INTEGRATION ---")
for target in target_blocks:
    bx, by, bz = target["base_coords"]
    send_to_ik_solver(target["color"], bx, by, bz)

# -----------------------------------------------------------------------
# DISPLAY AND SAVE IMAGE OUTPUT
# -----------------------------------------------------------------------

# Option 1: Save output image file to disk
save_path = "output_detected.jpg"
cv2.imwrite(save_path, output)
print(f"\n[INFO] Marked image saved to: {save_path}")

# Option 2: Render in Matplotlib (Ideal for Jupyter / Python Notebooks)
img_rgb = cv2.cvtColor(output, cv2.COLOR_BGR2RGB)
plt.figure(figsize=(10, 6))
plt.imshow(img_rgb)
plt.title("Detected Objects with Depth & Coordinates")
plt.axis("off")
plt.show()

# Option 3: Standard OpenCV Window
cv2.imshow("Marked Objects with Depth", output)
cv2.waitKey(0)
cv2.destroyAllWindows()