import cv2
import numpy as np
import math

# -----------------------------------------------------------------------
# IMPORT YOUR IK SOLVER (Or ensure ik_solver.py is in the same directory)
# -----------------------------------------------------------------------
from IKtest import solve_ik 

# -----------------------------------------------------------------------
# CALIBRATION PARAMETERS (IN CENTIMETERS)
# -----------------------------------------------------------------------
REAL_OBJECT_WIDTH_CM = 2.0  # Real physical width of your block (2.0 cm)
BLOCK_HEIGHT_CM = 2.0       # Physical height of the block (places grasp center at +1 cm)

# Lowered default focal length: standard webcam wide field-of-view (~300-350 px)
# Lowering this brings Z_cam (forward depth) back from 26+ cm down to 10-15 cm.
FOCAL_LENGTH_PX = 320.0  

# Focal Length Recalibration Mode:
# Set to True and place a 2cm block at KNOWN_CALIBRATION_DISTANCE_CM to get exact f_px
CALIBRATE_FOCAL_LENGTH = False 
KNOWN_CALIBRATION_DISTANCE_CM = 15.0  # Measured distance from lens to block front face

# -----------------------------------------------------------------------
# ROBOT BASE TRANSFORM (IN CENTIMETERS)
# Measured physically from camera lens center to arm base rotation axis origin.
# -----------------------------------------------------------------------
CAMERA_OFFSET_X_CM = 0.0   # Lateral displacement (Left/Right) relative to base
CAMERA_OFFSET_Y_CM = 5.0   # Forward displacement (Distance lens is in FRONT of base)
CAMERA_HEIGHT_Z_CM = 18.0  # Height of camera lens GLASS directly above TABLE plane

# -----------------------------------------------------------------------
# DETECTION & KINEMATIC MAPPING PIPELINE
# -----------------------------------------------------------------------
def process_vision_frame(img_path, hsv_masks):
    img = cv2.imread(img_path)
    if img is None:
        raise FileNotFoundError(f"Could not read image at {img_path}")

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_h, img_w = img.shape[:2]
    cx, cy = img_w / 2.0, img_h / 2.0

    # Ignore top 45% (robotic arm & overhead wires)[cite: 1]
    roi_mask = np.zeros((img_h, img_w), dtype=np.uint8)
    roi_mask[int(img_h * 0.45):, :] = 255

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    output = img.copy()

    print("\n--- DETECTED OBJECT COORDINATES & IK TARGETS ---")

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
                # 1. Automatic Focal Length Calibrator
                if CALIBRATE_FOCAL_LENGTH:
                    calibrated_f = (w * KNOWN_CALIBRATION_DISTANCE_CM) / REAL_OBJECT_WIDTH_CM
                    print(f"[CALIBRATION MODE] Detected block width = {w} px.")
                    print(f"--> Set FOCAL_LENGTH_PX = {calibrated_f:.2f} in your constants!\n")
                    continue

                # 2. Bounding Box Center Pixel
                center_x = x + (w / 2.0)
                center_y = y + (h / 2.0)

                # 3. Camera Coordinates (Centimeters relative to Lens)
                Z_cam_cm = (FOCAL_LENGTH_PX * REAL_OBJECT_WIDTH_CM) / float(w)
                X_cam_cm = ((center_x - cx) * Z_cam_cm) / FOCAL_LENGTH_PX
                
                # Vertical angle calculation from principal line
                Y_cam_cm = ((center_y - cy) * Z_cam_cm) / FOCAL_LENGTH_PX

                # 4. Transform to Robot Coordinate Frame (Centimeters)
                # ik_x = Forward distance from base axis
                # ik_y = Lateral offset (+ left / - right)
                # ik_z = Target grasp height relative to table base plane
                ik_x = Z_cam_cm + CAMERA_OFFSET_Y_CM
                ik_y = X_cam_cm + CAMERA_OFFSET_X_CM
                
                # FIXED HEIGHT CALCULATION:
                # Places Z target at block grasp height (~1.0cm to 2.0cm above table surface)
                table_relative_z = CAMERA_HEIGHT_Z_CM - Y_cam_cm
                ik_z = max(0.5, min(table_relative_z, BLOCK_HEIGHT_CM))

                print(f"[{color_name} Block]")
                print(f"  Camera Frame (cm) -> X_cam: {X_cam_cm:+.2f}, Y_cam: {Y_cam_cm:+.2f}, Z_cam: {Z_cam_cm:.2f}")
                print(f"  IK Target    (cm) ->    x: {ik_x:+.2f},    y: {ik_y:+.2f},    z: {ik_z:.2f}")

                # 5. Solve Inverse Kinematics
                try:
                    b_cmd, s_cmd, e_cmd = solve_ik(ik_x, ik_y, ik_z)
                    print(f"  IK Solution       -> Base: {b_cmd:.1f}°, Shoulder: {s_cmd:.1f}°, Elbow: {e_cmd:.1f}°\n")
                except ValueError as err:
                    print(f"  IK Solution       -> Unreachable target ({err})\n")

                # Visual Overlay
                cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(output, (int(center_x), int(center_y)), 3, (0, 0, 0), -1)
                cv2.putText(output, f"{color_name} ({ik_x:.1f},{ik_y:.1f},{ik_z:.1f})cm", 
                            (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

    cv2.imshow("Detected IK Targets", output)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# -----------------------------------------------------------------------
# EXECUTION
# -----------------------------------------------------------------------
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