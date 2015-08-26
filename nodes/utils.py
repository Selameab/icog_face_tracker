import cv2


class FaceDetect:
    def __init__(self, cascade_file_path="/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml"):
        self.face_cascade = cv2.CascadeClassifier(cascade_file_path)

    def detect(self, frame):
        return self.face_cascade.detectMultiScale(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 1.1, 2)
