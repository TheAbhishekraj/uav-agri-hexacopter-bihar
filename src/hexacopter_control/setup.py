from setuptools import find_packages, setup

package_name = 'hexacopter_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhishek Raj',
    maintainer_email='rajabhi2602@gmail.com',
    description='Offboard control for Bihar Agri-Hexacopter',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'flight_controller = hexacopter_control.flight_controller:main',
        ],
    },
)