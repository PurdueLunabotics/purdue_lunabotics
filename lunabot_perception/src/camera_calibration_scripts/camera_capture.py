import cv2
import os
from camera_calibration import calibrate_camera

GRID_SIZE = (9, 6)

# Default port, run detect_camera to find cameras on other ports
CAMERA_PORT = 0

CAMERA_NAME = "front_camera"

# Termination critieria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

SHOULD_CALIBRATE_AFTER = True

def capture_loop():

    #Whether to draw all captured corners (T) or just the most recent set (F)
    draw_collected_corners = True

    camera = cv2.VideoCapture(CAMERA_PORT)
    image_count = 0

    # Create the directory for writing captured images to -->
    script_dir_path = os.path.dirname(os.path.abspath(__file__)) + "/captured_images"   

    image_dir = ""
    dir_count = 1
    while True:
        try:
            image_dir = os.path.join(script_dir_path, CAMERA_NAME + "_calibration" + str(dir_count))
            os.makedirs(image_dir)
            break
        except:
            dir_count += 1
    # <-- 

    
    collected_corners = []

    
    while True:
        ret, img = camera.read()
        save_copy = img.copy()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(gray, GRID_SIZE, None)
        if found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        cv2.drawChessboardCorners(img, GRID_SIZE, corners, ret)

        # Draw the other collected corners if we want to, we will collect the most recent capture later
        if draw_collected_corners:
            for collected in collected_corners:
                cv2.drawChessboardCorners(img, GRID_SIZE, collected, ret)
       


        # Display the result
        cv2.imshow("Collected Corners", img)

        # Detect a key being pressed
        key = cv2.waitKey(1) & 0xFF

        if key == ord("c"):
            image_count += 1
            cv2.imwrite(image_dir + f"/img{image_count}.jpg", save_copy)
            
            #save the corners for display
            collected_corners.append(corners)
        # toggle drawing corners
        elif key == ord("t"):
            draw_collected_corners = not draw_collected_corners
        # quit
        elif key == ord("q"):
            if image_count > 0:
                print("Wrote images to " + image_dir)
                return (CAMERA_NAME, dir_count)
            else:
                print("could not write images")
            return (" ", -1)


if __name__ == "__main__":
    camera_name, cal_num = capture_loop()
    if (SHOULD_CALIBRATE_AFTER):
        calibrate_camera(camera_name, cal_num)