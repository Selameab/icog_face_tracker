#!/usr/bin/python
__author__ = 'ss'

import rospy
import cv2
from sensor_msgs.msg import Image
from icog_face_tracker.msg import facebox
from icog_face_tracker.msg import faces
from cv_bridge import CvBridge, CvBridgeError
from utils import FaceDetect


class Tracker:
    NODE_NAME = 'tracker_node'

    def __init__(self):
        self.window = rospy.get_param("/" + self.NODE_NAME + "/window")
        self.cv_bridge = CvBridge()
        self.face_publisher = rospy.Publisher("/faces", faces, queue_size=10)
        self.img_subscriber = rospy.Subscriber("/usb_cam_node/image_raw", Image, self.callback)
        self.face_detector = FaceDetect()
        self.faces = []
        self.faces_msg = None
        cv2.namedWindow("Live Feed")

    def callback(self, data):
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")  # Convert ROS image to CVMat
            self.faces = self.face_detector.detect(frame)  # Get coordinates of faces in frame
            # Generate "/faces"
            self.faces_msg = faces()
            self.faces_msg.face_boxes = []
            self.faces_msg.img_height.data = frame.shape[0]
            self.faces_msg.img_width.data = frame.shape[1]
            for (x, y, w, h) in self.faces:
                new_face = facebox()
                new_face.top_left.x = x
                new_face.top_left.y = y
                new_face.width_height.x = w
                new_face.width_height.y = h
                self.faces_msg.face_boxes.append(new_face)
            self.face_publisher.publish(self.faces_msg)

            # Display window if param /window is true
            if self.window:
                for (x, y, w, h) in self.faces:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 180, 50), 1)
                cv2.imshow("Live Feed", frame)
                cv2.waitKey(1)

        except CvBridgeError, e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node(Tracker.NODE_NAME, anonymous=False)
    rospy.loginfo("Starting " + Tracker.NODE_NAME)
    Tracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Stopping " + Tracker.NODE_NAME)
