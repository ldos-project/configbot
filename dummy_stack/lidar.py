#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import argparse

class FakeLidarNode:
    def __init__(self, hz):
        self.pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        self.angle_min = -1.57
        self.angle_max = 1.57
        self.angle_increment = np.deg2rad(0.1)
        self.range_min = 0.2
        self.range_max = 10.0
        self.num_points = int((self.angle_max - self.angle_min) / self.angle_increment)
        self.timer = rospy.Timer(rospy.Duration(1.0 / hz), self.publish_scan)

    def publish_scan(self, _):
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_laser"
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max


        # Randomly decide whether to simulate an obstacle in front
        obstacle = np.random.rand() < 0.5  # 50% chance of obstacle

        scan.ranges = []
        for i in range(self.num_points):
            angle = self.angle_min + i * self.angle_increment
            if abs(angle) < 0.2:
                r = 1.2 + np.random.normal(0, 0.05) if obstacle else 10.0  # obstacle at ~1.2m
            else:
                r = 10.0
            scan.ranges.append(r)

        self.pub.publish(scan)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--hz', type=float, default=60.0, help='Publishing frequency')
    args, _ = parser.parse_known_args()

    rospy.init_node('fake_lidar_node')
    FakeLidarNode(hz=args.hz)
    rospy.spin()