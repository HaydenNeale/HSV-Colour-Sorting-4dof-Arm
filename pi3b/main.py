import cv2
import numpy as np

img_path = r"C:\Users\ender\PyCharmMiscProject\HSVcv\lego1-clear.jpg"  # r makes it a raw string, prevents weird issues when pasting location

img = cv2.imread(img_path)
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