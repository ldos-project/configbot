#!/usr/bin/env python3
import rospy
import numpy as np
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FakeCamera:
    def __init__(self, hz):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / hz), self.timer_cb)
        self.width, self.height = 1920, 1080

    def timer_cb(self, _):
        img = np.random.randint(0, 256, (self.height, self.width, 3), dtype=np.uint8)
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.pub.publish(msg)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--hz', type=float, default=60.0, help='Publishing frequency')
    args, _ = parser.parse_known_args()

    rospy.init_node('fake_camera_node')
    f = FakeCamera(hz=args.hz)
    rospy.spin()