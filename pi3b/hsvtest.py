import cv2
import numpy as np

img_path = r"models-pictures\img1.jpg"  # relative to the working directory (project root)

# -----------------------------------------------------------------------
# CALIBRATION MODE
# Set to "Red", "Blue", "Green", "Yellow", or None to skip tuning and just
# run detection with whatever values are currently set below.
# -----------------------------------------------------------------------
TUNE_COLOR = "Red"

img = cv2.imread(img_path)
if img is None:
    raise FileNotFoundError(f"Could not read image at {img_path}")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# colour ranges for HSV
lower_red1 = np.array([0, 100, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 100, 70])
upper_red2 = np.array([179, 255, 255])

lower_blue = np.array([75, 15, 10])
upper_blue = np.array([140, 255, 255])

lower_green = np.array([35, 30, 10])
upper_green = np.array([85, 255, 255])

lower_yellow = np.array([15, 80, 80])
upper_yellow = np.array([35, 255, 255])


def tune_hsv_range(hsv_img, window_name, lower_init, upper_init):
    """
    Opens a trackbar window over hsv_img. Drag sliders until only the target
    block is white in the mask. Press 'q' to confirm and return the tuned
    (lower, upper) np.array pair.
    """
    cv2.namedWindow(window_name)
    cv2.createTrackbar("H min", window_name, int(lower_init[0]), 179, lambda x: None)
    cv2.createTrackbar("H max", window_name, int(upper_init[0]), 179, lambda x: None)
    cv2.createTrackbar("S min", window_name, int(lower_init[1]), 255, lambda x: None)
    cv2.createTrackbar("S max", window_name, int(upper_init[1]), 255, lambda x: None)
    cv2.createTrackbar("V min", window_name, int(lower_init[2]), 255, lambda x: None)
    cv2.createTrackbar("V max", window_name, int(upper_init[2]), 255, lambda x: None)

    print(f"\n[{window_name}] Adjust sliders until only the target block(s) show white "
          f"in the Mask window. Press 'q' to confirm.")

    while True:
        h_min = cv2.getTrackbarPos("H min", window_name)
        h_max = cv2.getTrackbarPos("H max", window_name)
        s_min = cv2.getTrackbarPos("S min", window_name)
        s_max = cv2.getTrackbarPos("S max", window_name)
        v_min = cv2.getTrackbarPos("V min", window_name)
        v_max = cv2.getTrackbarPos("V max", window_name)

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])

        mask = cv2.inRange(hsv_img, lower, upper)
        preview = cv2.bitwise_and(img, img, mask=mask)

        cv2.imshow(window_name, preview)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cv2.destroyWindow(window_name)
    cv2.destroyWindow("Mask")
    print(f"[{window_name}] Tuned range -> lower={lower.tolist()}, upper={upper.tolist()}")
    return lower, upper


# Run the tuner for whichever colour is selected, overwriting its range(s)
# before the detection pipeline below uses them.
if TUNE_COLOR == "Red":
    # Red wraps around hue 0/180, so tune both halves separately.
    lower_red1, upper_red1 = tune_hsv_range(hsv, "Tune Red (low half, hue near 0)", lower_red1, upper_red1)
    lower_red2, upper_red2 = tune_hsv_range(hsv, "Tune Red (high half, hue near 179)", lower_red2, upper_red2)
elif TUNE_COLOR == "Blue":
    lower_blue, upper_blue = tune_hsv_range(hsv, "Tune Blue", lower_blue, upper_blue)
elif TUNE_COLOR == "Green":
    lower_green, upper_green = tune_hsv_range(hsv, "Tune Green", lower_green, upper_green)
elif TUNE_COLOR == "Yellow":
    lower_yellow, upper_yellow = tune_hsv_range(hsv, "Tune Yellow", lower_yellow, upper_yellow)

# separate masks per color
masks = {
    "Red":    [(lower_red1, upper_red1), (lower_red2, upper_red2)],
    "Blue":   [(lower_blue, upper_blue)],
    "Green":  [(lower_green, upper_green)],
    "Yellow": [(lower_yellow, upper_yellow)],
}
labels = {
    "Red": (0, 0, 255),
    "Blue": (255, 0, 0),
    "Green": (0, 255, 0),
    "Yellow": (0, 255, 255),
}

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (22, 22))
MIN_AREA = 800  # tune this to your image resolution — filters out noise specks

output = img.copy()
combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

for color_name, ranges in masks.items():
    mask = None
    for lower, upper in ranges:
        m = cv2.inRange(hsv, lower, upper)
        mask = m if mask is None else (mask | m)

    clean_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    clean_mask = cv2.morphologyEx(clean_mask, cv2.MORPH_OPEN, kernel)  # removes small noise blobs
    combined_mask |= clean_mask

    contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(output, (x, y), (x + w, y + h), labels[color_name], 2)
        cv2.putText(output, color_name, (x, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, labels[color_name], 2)

result_filtered = cv2.bitwise_and(img, img, mask=combined_mask)

cv2.imshow("Original", img)
cv2.imshow("Filtered (mask applied)", result_filtered)
cv2.imshow("Marked objects", output)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Print final values for every colour so you can copy them back into the
# constants above once you're happy, even for colours you didn't tune this run.
print("\n--- Current HSV ranges ---")
print(f"lower_red1, upper_red1 = {lower_red1.tolist()}, {upper_red1.tolist()}")
print(f"lower_red2, upper_red2 = {lower_red2.tolist()}, {upper_red2.tolist()}")
print(f"lower_blue, upper_blue = {lower_blue.tolist()}, {upper_blue.tolist()}")
print(f"lower_green, upper_green = {lower_green.tolist()}, {upper_green.tolist()}")
print(f"lower_yellow, upper_yellow = {lower_yellow.tolist()}, {upper_yellow.tolist()}")