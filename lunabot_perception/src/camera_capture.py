import cv2
import os

grid_size = (9, 6)

def capture_loop():
    
    camera = cv2.VideoCapture(0)
    count = 0
    dir_count = 1

    script_dir = os.path.dirname(os.path.abspath(__file__))

    image_dir = ""
    while True:
        try:
            image_dir = os.path.join(script_dir, f"captured_images{dir_count}")
            os.makedirs(image_dir)
            break
        except:
            dir_count += 1


    while (True):
        ret, img = camera.read()
        save_copy = img.copy()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(gray, grid_size, None)
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # Draw the corners onto the original image
            cv2.drawChessboardCorners(img, grid_size, corners, ret)
            
            # Display the result
            cv2.imshow('Collected Corners', img)
        else:
            cv2.imshow('Collected Corners', img)
            print("Corners could not be found.")

        key = cv2.waitKey(1) & 0xFF

        if (key == ord('c')):
            count += 1
            cv2.imwrite(image_dir + f"/img{count}.jpg", save_copy)
        if (key == ord('q')):
            if (count > 0):
                print("Wrote images to " + image_dir)
            else:
                print("could not write images")
            break



if __name__ == "__main__":
    capture_loop()