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
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro') + glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf', 'mech'), glob('urdf/mech/*.xacro') + glob('urdf/mech/*.urdf')),
        (os.path.join('share', package_name, 'urdf', 'controllers'), glob('urdf/controllers/*.xacro') + glob('urdf/controllers/*.urdf')),
        (os.path.join('share', package_name, 'urdf', 'robots'), glob('urdf/robots/*.xacro') + glob('urdf/robots/*.urdf')),
        (os.path.join('share', package_name, 'urdf', 'sensors'), glob('urdf/sensors/*.xacro') + glob('urdf/sensors/*.urdf')),

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
            'test_cmdvel = src.test_cmdvel:main',
            'test_opencv = src.test_opencv:main',
        ],
    },
)
