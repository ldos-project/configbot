#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import String, Float64
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2
import numpy as np

class NavNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_saved = 0
        self.save_interval = 5.0  # seconds

        rospy.Subscriber('/scan/pointcloud', PointCloud2, self.pc_cb)
        rospy.Subscriber('/camera/image_raw', Image, self.image_cb)

        self.nav_pub = rospy.Publisher('/nav/decision', String, queue_size=10)
        self.fft_pub = rospy.Publisher('/nav/fft_energy', Float64, queue_size=10)
        
    def pc_cb(self, pc_msg):
        points = pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True)
        danger_zone = [p for p in points if abs(p[1]) < 0.2 and 0.0 < p[0] < 1.5]

        decision = "obstacle" if danger_zone else "clear"
        self.nav_pub.publish(decision)
        rospy.loginfo_throttle(1.0, f"[NAV] Decision: {decision} ({len(danger_zone)} close pts)")

    def image_cb(self, img_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')

            # 2D FFT and magnitude spectrum
            freq_domain = np.fft.fft2(img)
            spectrum = np.abs(freq_domain)
            energy = np.sum(spectrum)

            self.fft_pub.publish(energy)

        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[CAMERA] Decode error: {e}")

if __name__ == '__main__':
    rospy.init_node('nav_node')
    NavNode()
    rospy.spin()