import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class SobelEdgeDetectionNode(Node):
    def __init__(self):
        super().__init__('sobel_edge_detection_node')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribe to image_raw topic
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Publisher for sobel edge detection output
        self.publisher = self.create_publisher(
            Image,
            '/sobel_detection',
            10
        )

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Apply Sobel edge detection
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        sobel = np.sqrt(sobelx**2 + sobely**2)
        sobel = np.uint8(255 * sobel / np.max(sobel))

        # Convert OpenCV image to ROS Image
        sobel_msg = self.bridge.cv2_to_imgmsg(sobel, encoding='mono8')

        # Publish the processed image
        self.publisher.publish(sobel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SobelEdgeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()