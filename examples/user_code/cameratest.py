import cv2, time
import numpy as np
from imutils.video import FPS

print("waitforcap..")
#cap = cv2.VideoCapture(' rtspsrc location = rtsp://admin:admin123@192.168.1.108/  ! decodebin ! videoconvert ! appsink max-buffers=1 drop=true', cv2.CAP_GSTREAMER)
#cap = cv2.VideoCapture("rtsp://admin:admin123@192.168.1.108")
#cap = cv2.VideoCapture("http://admin:admin123@192.168.1.108/mjpeg.cgi?user=admin&password=admin123&channel=0&.mjpg")
#cap = cv2.VideoCapture("http:///admin:admin123@192.168.1.108/axis-cgi/mjpg/video.cgi")
#cap = cv2.VideoCapture("http://admin:admin123@192.168.1.108/axis-cgi/mjpg/video.cgi")
#cap = cv2.VideoCapture("http://admin:admin123@192.168.1.108/video.cgi?.mjpg")
cap = cv2.VideoCapture("http://admin:admin123@192.168.1.108/cgi-bin/mjpg/video.cgi?channel=1&subtype=1")
#cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
print(f"Buffersize: {cv2.CAP_PROP_BUFFERSIZE}")
print("cap opened")


if cap.isOpened():
    print("cam found")


fps = FPS().start()
framecount = 0

while(True):
    ret, frame = cap.read()
    #frame = cv2.resize(frame, (1280, 720))
    #time.sleep(1.2)
    if not ret:
        print("Frame not received")
        break
        #continue
    else:
        cv2.imshow('frame',frame)
        fps.update()
    #cap.grab()
    framecount += 1 
    if cv2.waitKey(1) & 0xFF == ord('q'):
        fps.stop()
        print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
        print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
        cap.release()
        cv2.destroyAllWindows()

        break


