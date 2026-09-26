import cv2

for i in range(0, 100):
    cam = cv2.VideoCapture(i)
    ret, img = cam.read()
    if ret:
        print("Found cam at: " + str(i))
        # Release the camera so other applications (or loops) can use it
        cam.release() 