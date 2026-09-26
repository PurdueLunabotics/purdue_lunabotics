import cv2

ports = []

for i in range(0, 100):
    cam = cv2.VideoCapture(i)
    ret, img = cam.read()
    if ret:
        print("Found cam at: " + str(i))
        ports.append(i)
        # Release the camera so other applications (or loops) can use it
        cam.release()
    else:
        print()
        print(f"\033[1mFound camera(s) at {', '.join(map(str, ports))}\033[0m")
        break