# import the necessary packages
from threading import Thread
import cv2
from imutils.video import FPS

class WebcamVideoStream:
	def __init__(self, src=0):
		# initialize the video camera stream and read the first frame
		# from the stream
		self.stream = cv2.VideoCapture(src)
		(self.grabbed, self.frame) = self.stream.read()
		# initialize the variable used to indicate if the thread should
		# be stopped
		self.stopped = False

	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		return self

	def update(self):
		# keep looping infinitely until the thread is stopped
		while True:
			# if the thread indicator variable is set, stop the thread
			if self.stopped:
				return
			# otherwise, read the next frame from the stream
			(self.grabbed, self.frame) = self.stream.read()

	def read(self):
		# return the frame most recently read
		return self.frame

	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True

# created a *threaded* video stream, allow the camera sensor to warmup,
# and start the FPS counter
print("[INFO] sampling THREADED frames from webcam...")
vs = WebcamVideoStream(src="rtsp://admin:admin123@192.168.1.108/").start()
fps = FPS().start()

# loop over some frames...this time using the threaded stream
while(True):
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 400 pixels
	frame = vs.read()
	#frame = imutils.resize(frame, width=400)
	
	# check to see if the frame should be displayed to our screen
	cv2.imshow("Frame", frame)
	#key = cv2.waitKey(1) & 0xFF
	# update the FPS counter
	fps.update()

	if cv2.waitKey(1) & 0xFF == ord('q'):
		# stop the timer and display FPS information
		fps.stop()
		print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
		print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
		# do a bit of cleanup
		cv2.destroyAllWindows()
		vs.stop()
		break