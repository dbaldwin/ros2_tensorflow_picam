import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
import importlib.util
from ament_index_python.packages import get_package_share_directory

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Declare parameters
        self.declare_parameter('model_dir', 'resource/tflite_model')
        self.declare_parameter('graph_name', 'detect.tflite')
        self.declare_parameter('labels', 'labelmap.txt')
        self.declare_parameter('threshold', 0.5)
        self.declare_parameter('use_tpu', False)

        # Load parameters
        package_share_directory = get_package_share_directory('ros2_tensorflow_picam')
        model_dir = os.path.join(package_share_directory, self.get_parameter('model_dir').get_parameter_value().string_value)
        graph_name = self.get_parameter('graph_name').get_parameter_value().string_value
        labels_path = self.get_parameter('labels').get_parameter_value().string_value
        self.min_conf_threshold = self.get_parameter('threshold').get_parameter_value().double_value
        use_tpu = self.get_parameter('use_tpu').get_parameter_value().bool_value

        # Load TensorFlow Lite model
        pkg = importlib.util.find_spec('tflite_runtime')
        if pkg:
            from tflite_runtime.interpreter import Interpreter
            if use_tpu:
                from tflite_runtime.interpreter import load_delegate
        else:
            from tensorflow.lite.python.interpreter import Interpreter
            if use_tpu:
                from tensorflow.lite.python.interpreter import load_delegate

        # Load the model
        model_path = os.path.join(model_dir, graph_name)
        self.interpreter = Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()

        # Load label map
        with open(os.path.join(model_dir, labels_path), 'r') as f:
            self.labels = [line.strip() for line in f.readlines()]
        if self.labels[0] == '???':
            del self.labels[0]

        # Get model input details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]

        # Setup ROS2 components
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.object_publisher = self.create_publisher(String, 'detected_objects', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame_resized = cv2.resize(frame, (self.width, self.height))
        input_data = np.expand_dims(frame_resized, axis=0)

        # Normalize the image
        floating_model = (self.input_details[0]['dtype'] == np.float32)
        if floating_model:
            input_data = (np.float32(input_data) - 127.5) / 127.5

        # Run inference
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        # Get results
        boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]  # Bounding box coordinates
        classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]  # Class index
        scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]  # Confidence scores

        # Publish detections
        for i in range(len(scores)):
            if scores[i] > self.min_conf_threshold:
                label = f"{self.labels[int(classes[i])]}: {scores[i]:.2f}"
                msg = String()
                msg.data = label
                self.object_publisher.publish(msg)

        self.get_logger().info(f'Detected {len(scores)} objects')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
