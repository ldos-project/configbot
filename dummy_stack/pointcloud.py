#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2

class LidarProcessorNode:
    def __init__(self):
        self.lg = lg.LaserProjection()
        self.pub = rospy.Publisher('/scan/pointcloud', PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.min_dist_pub = rospy.Publisher('/scan/min_obstacle_distance', Float32, queue_size=10)

    def scan_cb(self, scan_msg):
        pc2_msg = self.lg.projectLaser(scan_msg)
        pc2_msg.header.frame_id = "base_laser"

        # Compute distances
        points = pc2.read_points(pc2_msg, field_names=("x", "y", "z"), skip_nans=True)
        min_dist = float('inf')
        for x, y, z in points:
            dist = (x**2 + y**2 + z**2)**0.5
            if dist < min_dist:
                min_dist = dist

        self.min_dist_pub.publish(min_dist)
        self.pub.publish(pc2_msg)

if __name__ == '__main__':
    rospy.init_node('lidar_processor_node')
    LidarProcessorNode()
    rospy.spin()
