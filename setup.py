from setuptools import find_packages, setup
import os

package_name = 'ros2_tensorflow_picam'

# Include the model files in the install
model_files = []
resource_dir = os.path.join('resource', 'tflite_model')
for root, _, files in os.walk(resource_dir):
    for file in files:
        model_files.append(os.path.join(root, file))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource/tflite_model', model_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dexi',
    maintainer_email='db@droneblocks.io',
    description='ROS2 package for TensorFlow Lite object detection',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection_node = ros2_tensorflow_object_detection.object_detection_node:main',
        ],
    },
)
