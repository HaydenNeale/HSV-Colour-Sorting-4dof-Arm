import cv2
import numpy as np
import math

from IKtest import solve_ik 

# -----------------------------------------------------------------------
# CALIBRATION PARAMETERS
# -----------------------------------------------------------------------
REAL_OBJECT_WIDTH_CM = 2.0
FOCAL_LENGTH_PX = 320.0  

# -----------------------------------------------------------------------
# ROBOT BASE & CAMERA TRANSFORM (IN CENTIMETERS & DEGREES)
# -----------------------------------------------------------------------
CAMERA_OFFSET_X_CM = 0.0     # Lateral displacement (Left/Right) relative to base
CAMERA_OFFSET_Y_CM = 5.0     # Forward displacement in front of base axis
CAMERA_HEIGHT_Z_CM = 18.0    # Height of camera lens GLASS above TABLE/BASE plane

# CAMERA PITCH ANGLE (Degrees):
# 0.0  = Pointing straight forward horizontally
# 30.0 = Tilted 30 degrees DOWN toward the table
# Measure or estimate this angle (start around 25-40° for standard overhead mounts)
CAMERA_PITCH_DEG = 35.0  


def process_vision_frame(img_path, hsv_masks):
    img = cv2.imread(img_path)
    if img is None:
        raise FileNotFoundError(f"Could not read image at {img_path}")

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_h, img_w = img.shape[:2]
    cx, cy = img_w / 2.0, img_h / 2.0

    roi_mask = np.zeros((img_h, img_w), dtype=np.uint8)
    roi_mask[int(img_h * 0.35):, :] = 255

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    output = img.copy()

    # Convert pitch angle to radians
    pitch_rad = math.radians(CAMERA_PITCH_DEG)
    cos_p = math.cos(pitch_rad)
    sin_p = math.sin(pitch_rad)

    print("\n--- TILT-CORRECTED 3D IK TARGETS ---")

    for color_name, ranges in hsv_masks.items():
        mask = None
        for lower, upper in ranges:
            m = cv2.inRange(hsv, lower, upper)
            mask = m if mask is None else (mask | m)

        mask = cv2.bitwise_and(mask, roi_mask)
        clean_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) < 200:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h

            if 0.5 <= aspect_ratio <= 2.0:
                center_x = x + (w / 2.0)
                center_y = y + (h / 2.0)

                # 1. Unrotated Camera Coordinates (Optical Axis relative)
                Z_c = (FOCAL_LENGTH_PX * REAL_OBJECT_WIDTH_CM) / float(w)
                X_c = ((center_x - cx) * Z_c) / FOCAL_LENGTH_PX
                Y_c = ((center_y - cy) * Z_c) / FOCAL_LENGTH_PX

                # 2. Rotate Frame by Camera Pitch Angle (Tilted Downward)
                # Forward depth rotated relative to ground
                Y_rotated_forward = Y_c * sin_p + Z_c * cos_p 
                # Vertical height drop rotated relative to ground
                Z_rotated_down    = Y_c * cos_p - Z_c * sin_p 

                # 3. Translate to Robot Base Origin (Centimeters)
                ik_x = Y_rotated_forward + CAMERA_OFFSET_Y_CM
                ik_y = X_c + CAMERA_OFFSET_X_CM
                
                # Height above base plane (accounts for camera angle)
                ik_z = CAMERA_HEIGHT_Z_CM - Z_rotated_down

                print(f"[{color_name} Block]")
                print(f"  Raw Cam Frame (cm) -> X: {X_c:+.2f}, Y: {Y_c:+.2f}, Z: {Z_c:.2f}")
                print(f"  Robot Target  (cm) -> x: {ik_x:+.2f}, y: {ik_y:+.2f}, z: {ik_z:+.2f}")

                # 4. Pass to IK Solver
                try:
                    b_cmd, s_cmd, e_cmd = solve_ik(ik_x, ik_y, ik_z)
                    print(f"  IK Commands        -> Base: {b_cmd:.1f}°, Shoulder: {s_cmd:.1f}°, Elbow: {e_cmd:.1f}°\n")
                except ValueError as err:
                    print(f"  IK Command Status  -> Target out of reach ({err})\n")

                # Visual Overlay
                cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(output, (int(center_x), int(center_y)), 3, (0, 0, 0), -1)
                cv2.putText(output, f"{color_name} Z:{ik_z:.1f}cm", 
                            (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

    cv2.imshow("Tilt-Corrected IK Targets", output)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    test_masks = {
        "Red": [
            (np.array([0, 130, 90]), np.array([10, 255, 255])),
            (np.array([170, 130, 90]), np.array([179, 255, 255]))
        ],
        "Blue": [(np.array([95, 120, 80]), np.array([130, 255, 255]))],
        "Green": [(np.array([45, 80, 60]), np.array([85, 255, 255]))],
        "Yellow": [(np.array([20, 120, 120]), np.array([35, 255, 255]))],
    }

    process_vision_frame(r"models-pictures\img1.jpg", test_masks)