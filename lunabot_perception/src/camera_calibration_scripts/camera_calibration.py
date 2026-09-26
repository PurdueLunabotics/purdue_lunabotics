import numpy as np
import cv2
import glob
import os

# ==========================================
# CONFIGURATION
# ==========================================
# Number of INSIDE corners on your checkerboard (Width, Height)
# If your board has 10x7 squares, the inside corners will be 9x6
CHECKERBOARD_SIZE = (9, 6)

# Real-world size of a single square edge (e.g., in millimeters or meters)
# Use 1.0 if you only care about the intrinsic matrix and pixel-space distortion
SQUARE_SIZE = 1.0



# ==========================================
# DATA_DIR = os.path.dirname(os.path.abspath(__file__)) + "/image_session_data"
# Write calibrations here
DATA_DIR = "lunabot_perception\\calibrations"



def calibrate_camera(camera_name: str, calibration_number: int):
    # Path to the folder containing your calibration images (e.g., .jpg, .png)
    IMAGES_DIR = (
        os.path.dirname(os.path.abspath(__file__)) + f"/captured_images/{camera_name}_calibration{calibration_number}"
    )

    # Termination criteria for sub-pixel corner refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare 3D object points in the real-world coordinate system
    # E.g., (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
    objp = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[
        0 : CHECKERBOARD_SIZE[0], 0 : CHECKERBOARD_SIZE[1]
    ].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    # Arrays to store object points and image points from all valid images
    object_points = []  # 3d point in real world space
    image_points = []  # 2d points in image plane.

    # Grab all images from the specified folder
    image_extensions = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
    images = []
    for ext in image_extensions:
        images.extend(glob.glob(os.path.join(IMAGES_DIR, ext)))

    if not images:
        print(
            f"Error: No images found in '{IMAGES_DIR}'. Please check the directory path."
        )
        return

    print(f"Found {len(images)} images. Processing...")

    gray = None
    valid_image_count = 0

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_SIZE, None)

        # If found, add object points, image points (after refining them)
        if ret:
            object_points.append(objp)
            valid_image_count += 1

            # Refine corner locations to sub-pixel accuracy
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            image_points.append(corners2)

            # Optional: Draw and display the corners to verify detection
            cv2.drawChessboardCorners(img, CHECKERBOARD_SIZE, corners2, ret)
            cv2.imshow("Chessboard Detection Preview", img)
            cv2.waitKey(100)  # Pause for 100ms per image
        else:
            print(f"Warning: Checkerboard corners not found in image: {fname}")

    cv2.destroyAllWindows()

    if valid_image_count < 10:
        print(
            f"\nWarning: Only {valid_image_count} images were valid. At least 10-20 distinct angles are recommended for good calibration."
        )

    if valid_image_count == 0:
        print(
            "Error: Could not find checkerboard corners in any of the provided images."
        )
        return

    print("\nRunning camera calibration optimization...")
    # Perform camera calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, gray.shape[::-1], None, None
    )

    print("\n=== Calibration Successful ===")
    print(f"Reprojection Error: {ret:.4f} pixels (Lower is better, ideal is < 0.5)")
    print("\nCamera Matrix (Intrinsic parameters):\n", mtx)
    print("\nDistortion Coefficients:\n", dist)

    if not os.path.isdir(DATA_DIR):
        os.makedirs(DATA_DIR)
    # Save the calibration parameters to a compressed numpy file for future use
    output_filename = (
            f"lunabot_perception/calibrations/{camera_name}_calibration_data{calibration_number}.npz"
    )
    np.savez(output_filename, mtx=mtx, dist=dist)
    print(f"\nParameters successfully saved to '{output_filename}'")

    # ==========================================
    # VERIFICATION: Undistort a sample image
    # ==========================================
    sample_img_path = images[0]
    img = cv2.imread(sample_img_path)
    h, w = img.shape[:2]

    # Refine the camera matrix based on free scaling parameter (alpha)
    new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    # Undistort the image
    undistorted_img = cv2.undistort(img, mtx, dist, None, new_camera_mtx)

    # Crop the image based on ROI to remove black padding boundaries
    x, y, w_box, h_box = roi
    undistorted_img_cropped = undistorted_img[y : y + h_box, x : x + w_box]

    # Display side-by-side comparison
    cv2.imshow("Original Image", img)
    cv2.imshow("Undistorted Image", undistorted_img_cropped)
    print("\nPress any key on the image windows to close and exit.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    calibrate_camera("front_camera", 1)
