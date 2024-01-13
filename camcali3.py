import cv2
import numpy as np

# List of file names of your calibration images
calibration_image_files = ['pic.jpg', 'pic2.jpg', 'pic3.jpg' , 'pic4.jpg', 'pic5.jpg', 'pic6.jpg', 'pic(1).jpg']

# Create a list to store calibration images
images = ['pic.jpg', 'pic2.jpg', 'pic3.jpg' , 'pic4.jpg', 'pic5.jpg', 'pic6.jpg', 'pic(1).jpg']

# Load calibration images
for file_name in calibration_image_files:
    img = cv2.imread("/home/luccy/projects /camera calibration/")
    if img is None:
        print(f"Error loading image: {file_name}")
        continue
    images.append(img)

for img, corners in zip(images, imgpoints):
    print(f"Number of corners in an image: {len(corners)}")




# Check if there are any valid images
if not images:
    print("No valid images found.")
    exit()

# Define the number of rows and columns on your chessboard
chessboard_rows = 7  # Change this to the actual number of rows on your chessboard
chessboard_cols = 10  # Change this to the actual number of columns on your chessboard

# Prepare object points (3D coordinates of chessboard corners)
objp = np.zeros((chessboard_rows * chessboard_cols, 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_rows, 0:chessboard_cols].T.reshape(-1, 2)

# Arrays to store object points and image points
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane

# Find chessboard corners in each image
for img in images:
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (chessboard_cols, chessboard_rows), None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

# Check if there are any valid calibration points
if not objpoints or not imgpoints:
    print("No valid calibration points found.")
    exit()

# Calibrate camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Print camera matrix and distortion coefficients
print("Camera Matrix:")
print(mtx)
print("\nDistortion Coefficients:")
print(dist)
