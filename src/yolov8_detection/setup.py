from setuptools import setup

package_name = 'yolov8_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhishek',
    maintainer_email='abhishek@example.com',
    description='YOLOv8 detection node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = yolov8_detection.detection_node:main',
            'thermal_ai = yolov8_detection.thermal_ai_node:main',
        ],
    },
)