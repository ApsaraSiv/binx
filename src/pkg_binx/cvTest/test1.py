import numpy as np
import cv2

camera = cv2.VideoCapture(0)

if (camera.isOpened()):
    print("camera opened")
else:
    print("could not open cam")


while True:  
     
    success, frame = camera.read()

    if not success:
        print("Can't read frame. End.")
        break


    frame2 = cv2.GaussianBlur(frame, (31,31), 0)
    combinedImages = np.hstack((frame,frame2))

    cv2.imshow('camera video', combinedImages)

    if cv2.waitKey(1) == ord('c'):
        break

camera.release()
cv2.destroyAllWindows()