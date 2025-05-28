#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from PIL import Image as PILImage
import torchvision
import torchvision.transforms as T
import torch
import cv2

class ObjectDetector:
    def __init__(self):
        self.bridge = CvBridge()

        # Load model + weights + labels
        weights = torchvision.models.detection.FasterRCNN_ResNet50_FPN_Weights.DEFAULT
        self.model = torchvision.models.detection.fasterrcnn_resnet50_fpn(weights=weights)
        self.model.eval()
        self.labels = weights.meta['categories']
        self.transform = weights.transforms()

        self.sub = rospy.Subscriber('/camera/image_raw', Image, self.image_cb)
        self.pub = rospy.Publisher('/detections', String, queue_size=10)

    def image_cb(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Convert OpenCV to PIL and apply transform
            pil_img = PILImage.fromarray(cv_img)
            tensor = self.transform(pil_img).unsqueeze(0)

            # Inference
            with torch.no_grad():
                output = self.model(tensor)[0]

            # Filter + publish detections
            detected = [self.labels[i] for i, s in zip(output['labels'], output['scores']) if s > 0.7]
            result = ', '.join(detected) if detected else 'none'
            self.pub.publish(result)

        except Exception as e:
            rospy.logwarn(f"[OBJDET] Error: {e}")

if __name__ == '__main__':
    rospy.init_node('object_detection_node')
    ObjectDetector()
    rospy.spin()
