import cv2
import numpy as np

# Prepare object points for an 8x6 checkerboard with 25mm squares
square_size = 0.025 # 2.5 cm
objp = np.zeros((6 * 8, 3), np.float32)
objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2) * square_size

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Add 10-15 images of a checkerboard held at different angles
images = ['calib1.jpg', 'calib2.jpg', ...] 

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (8, 6), None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

# Returns camera matrix containing [fx, 0, cx; 0, fy, cy; 0, 0, 1]
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

focal_length_x = mtx[0, 0]
focal_length_y = mtx[1, 1]
print(f"Calibrated Focal Length (fx): {focal_length_x:.2f} px")