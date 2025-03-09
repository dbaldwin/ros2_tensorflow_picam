import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CannyEdgeDetectionNode(Node):
    def __init__(self):
        super().__init__('canny_edge_detection_node')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribe to image_raw topic
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Publisher for canny edge detection output
        self.publisher = self.create_publisher(
            Image,
            '/canny_detection',
            10
        )

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Apply Canny edge detection
        edges = cv2.Canny(gray, 100, 200)

        # Convert OpenCV image to ROS Image
        canny_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')

        # Publish the processed image
        self.publisher.publish(canny_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CannyEdgeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
