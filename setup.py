from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'dome'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pitosalas',
    maintainer_email='pitosalas@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dome_controller = dome.dome_controller:main',
            'rotation_test = dome.rotation_test:main',
            'sensorcsv = dome.sensorcsv:main',
            'test_cmdvel = dome.test_cmdvel:main',
            'test_opencv = dome.test_opencv:main',
            'trace_imu = dome.trace_imu:main',
            'timestampmonitor = dome.timestampmonitor:main'
            'trace_sensor_data = dome.trace_sensor_data:main',
        ],
    },
)
