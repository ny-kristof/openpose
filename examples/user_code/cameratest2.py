
import cv2,threading, time
import queue as Queue

# bufferless VideoCapture
class VideoCapture:

  def __init__(self, name):
    self.cap = cv2.VideoCapture(name)
    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    self.q = Queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except Queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()

cap = VideoCapture('rtsp://admin:admin123@192.168.1.108/')
#cap = VideoCapture(0)
while True:
  frame = cap.read()
  #time.sleep(.01)   # simulate long processing
  cv2.imshow("frame", frame)
  if chr(cv2.waitKey(1)&255) == 'q':
    break